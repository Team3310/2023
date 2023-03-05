package org.frcteam2910.c2020;

import org.frcteam2910.c2020.commands.ArmRotationControlJoysticks;
import org.frcteam2910.c2020.commands.ArmTranslationalControlJoysticks;
import org.frcteam2910.c2020.commands.ChangeDriveMode;
import org.frcteam2910.c2020.commands.DriveBalanceCommand;
import org.frcteam2910.c2020.commands.VariableIntakeRPMCommand;
import org.frcteam2910.c2020.commands.ZeroAll;
import org.frcteam2910.c2020.commands.ArmExtenderZero;
import org.frcteam2910.c2020.subsystems.ArmExtender;
import org.frcteam2910.c2020.subsystems.ArmRotator;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.DriverReadout;
import org.frcteam2910.c2020.util.GyroAutoChooser;
import org.frcteam2910.c2020.util.SideChooser;
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
    private final GyroAutoChooser gyroAutoChooser;

    private final DriverReadout driverReadout;

    public RobotContainer() {
        sideChooser = new SideChooser();
        gyroAutoChooser = new GyroAutoChooser();

        autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, sideChooser.getSide());
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);

        drivetrain.setController(primaryController);
        intake.setController(secondaryController);

        driverReadout = new DriverReadout(this);

        CommandScheduler.getInstance().registerSubsystem(drivetrain);
        CommandScheduler.getInstance().registerSubsystem(intake);
        
        CommandScheduler.getInstance().setDefaultCommand(armRotator, new ArmRotationControlJoysticks(armRotator, getArmRotationAxis()));
        CommandScheduler.getInstance().setDefaultCommand(armExtender, new ArmTranslationalControlJoysticks(armExtender, getArmTranslationalAxis()));

        configureButtonBindings();
        
        instance = this;
    }

    public void updateTrajectoriesBasedOnSide(){
        autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, sideChooser.getSide());
        autonomousChooser.updateTrajectories(autonomousTrajectories);
        // SmartDashboard.putString("Side", sideChooser.getSide().toString());
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
                new InstantCommand(()->intake.setServoPosition(-1))
        );
        primaryController.getBButton().whenPressed(
                new InstantCommand(()->intake.setServoPosition(1))
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

        secondaryController.getBButton().onTrue(new InstantCommand(() -> {
            armRotator.setArmDegreesPositionAbsolute(30);
            armExtender.setTargetArmInchesPositionAbsolute(3.5);
        }));

        secondaryController.getAButton().onTrue(new InstantCommand(() -> {
            armRotator.setArmDegreesPositionAbsolute(0);
            armExtender.setTargetArmInchesPositionAbsolute(0);
        }));

        secondaryController.getXButton().onTrue(new InstantCommand(() -> {
            armRotator.setArmDegreesPositionAbsolute(-87);
            armExtender.setTargetArmInchesPositionAbsolute(0);
        }));

        secondaryController.getYButton().onTrue(new InstantCommand(() -> {
            armRotator.setArmDegreesPositionAbsolute(-105);
            armExtender.setTargetArmInchesPositionAbsolute(12);
        }));

        
        secondaryController.getStartButton().onTrue(new ArmExtenderZero(armExtender));
            
        // SmartDashboard.putData("Turn to Goal", new InstantCommand(() -> drivetrain.setTurnToTarget()));

        // SmartDashboard.putData("Limelight broken", new InstantCommand(() -> drivetrain.setLimelightOverride(true)));
        // SmartDashboard.putData("Limelight working", new InstantCommand(() -> drivetrain.setLimelightOverride(false)));
        SmartDashboard.putData("set arm to 30 degrees", new InstantCommand(() -> armRotator.setArmDegreesPositionAbsolute(30)));
        SmartDashboard.putData("set arm to 60 degrees", new InstantCommand(() -> armRotator.setArmDegreesPositionAbsolute(60)));
        SmartDashboard.putData("set arm to 90 degrees", new InstantCommand(() -> armRotator.setArmDegreesPositionAbsolute(90)));
        SmartDashboard.putData("set arm to 0 degrees", new InstantCommand(() -> armRotator.setArmDegreesPositionAbsolute(0)));
    }
    public Command getAutonomousCommand() {
        return autonomousChooser.getCommand(this);
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrain;
    }

    public GyroAutoChooser getGyroAutoAdjustMode() {
        return gyroAutoChooser;
    }

    public SideChooser getSideChooser(){
        return sideChooser;
    }

    public XboxController getPrimaryController() {
        return primaryController;
    }

    public XboxController getSecondaryController() {
        return secondaryController;
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
