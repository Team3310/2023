package org.frcteam2910.c2020;

import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.DriverReadout;
import org.frcteam2910.c2020.util.SideChooser;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.DPadButton;
import org.frcteam2910.common.robot.input.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class RobotContainer {
    private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
    private final XboxController secondaryController = new XboxController(Constants.SECONDARY_CONTROLLER_PORT);

    private static RobotContainer instance;

    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final Intake intake = Intake.getInstance();
    private final ArmRotator armRotator = ArmRotator.getInstance();
    private final ArmExtender armExtender = ArmExtender.getInstance();

    
    private AutonomousTrajectories autonomousTrajectories;
    private final AutonomousChooser autonomousChooser;
    private final SideChooser sideChooser;

    private final DriverReadout driverReadout;

    public RobotContainer() {
        sideChooser = new SideChooser();

        autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, sideChooser.getSide());
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);

        drivetrain.setController(primaryController);
        //intake.setController(secondaryController);

        driverReadout = new DriverReadout(this);

        CommandScheduler.getInstance().registerSubsystem(drivetrain);
        CommandScheduler.getInstance().setDefaultCommand(intake, new VariableIntakeRPMCommand(intake, getIntakeAxis(), getOuttakeAxis()));
        CommandScheduler.getInstance().setDefaultCommand(armRotator, new ArmRotationControlJoysticks(armRotator, getArmRotationAxis()));
        CommandScheduler.getInstance().setDefaultCommand(armExtender, new ArmTranslationalControlJoysticks(armExtender, getArmTranslationalAxis()));

        configureButtonBindings();
        
        instance = this;
    }

    public void updateTrajectoriesBasedOnSide(){
        autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, sideChooser.getSide());
        autonomousChooser.updateTrajectories(autonomousTrajectories);
        SmartDashboard.putString("Side", sideChooser.getSide().toString());
    }

    public void updateSide(){
        drivetrain.setSide(sideChooser.getSide());
    }

    private void configureButtonBindings() {
        
        primaryController.getStartButton().whenPressed(
                new InstantCommand(()->drivetrain.resetSteerAbsoluteAngle())
        );
        primaryController.getBackButton().whenPressed(
                new ZeroAll(drivetrain)
        );
        primaryController.getRightBumperButton().whenPressed(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.ROBOT_CENTRIC)
        );
        primaryController.getRightBumperButton().whenReleased(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
       primaryController.getLeftBumperButton().whenPressed(
               new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.LIMELIGHT)
       );
        primaryController.getLeftBumperButton().whenReleased(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
        primaryController.getYButton().whenPressed(
                new InstantCommand(()->intake.setServoSpeed(-1))
        );
        primaryController.getBButton().whenPressed(
                new InstantCommand(()->intake.setServoSpeed(1))
        );
        primaryController.getRightTriggerAxis().getButton(0.5).whenPressed(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.BALL_TRACK)
        );
        primaryController.getRightTriggerAxis().getButton(0.5).whenReleased(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );

        primaryController.getAButton().whenPressed(
            //new DriveBalanceCommand(drivetrain, false,true)
            new DriveBalanceCommand(drivetrain, false, true)
        );

        // primaryController.getXButton().whenPressed(
        //     new InstantCommand(()->drivetrain.setDriveControlMode(DriveControlMode.LIMELIGHT))
        // );

        //Misc
        secondaryController.getDPadButton(DPadButton.Direction.RIGHT).whenPressed(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
        secondaryController.getLeftJoystickButton().whenPressed(
                new InstantCommand(()-> drivetrain.setLimelightOverride(true))
        );
        secondaryController.getRightJoystickButton().whenPressed(
                new InstantCommand(()-> drivetrain.setLimelightOverride(false))
        );
            
        SmartDashboard.putData("Turn to Goal", new InstantCommand(() -> drivetrain.setTurnToTarget()));

        SmartDashboard.putData("Limelight broken", new InstantCommand(()-> drivetrain.setLimelightOverride(true)));
        SmartDashboard.putData("Limelight working", new InstantCommand(()-> drivetrain.setLimelightOverride(false)));
    }
    public Command getAutonomousCommand() {
        return autonomousChooser.getCommand(this);
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrain;
    }

    public SideChooser getSideChooser(){
        return sideChooser;
    }

    public XboxController getPrimaryController() {
        return primaryController;
    }

    private Axis getArmRotationAxis() {
        return secondaryController.getLeftYAxis();
    }
    private Axis getArmTranslationalAxis() {
        return secondaryController.getRightYAxis();
    }
    private Axis getIntakeAxis() {
        return secondaryController.getRightTriggerAxis();
    }
    private Axis getOuttakeAxis() {
        return secondaryController.getLeftTriggerAxis();
    }

    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }

    public DriverReadout getDriverReadout() {
        return driverReadout;
    }

    public Intake getIntake(){
        return intake;
    }

    public ArmRotator getArmRotator(){
        return armRotator;
    }

    public ArmExtender getArmExtender(){
        return armExtender;
    }

    public static RobotContainer getInstance() {
        if(instance == null) {
            //throw new RuntimeException("Somehow, RobotContainer constructor was never called and/or never set the RobotContainer instance");
        }
        return instance;
    }
}
