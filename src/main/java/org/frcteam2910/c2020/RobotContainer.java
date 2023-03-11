package org.frcteam2910.c2020;

import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.*;
import org.frcteam2910.common.robot.input.*;

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
        primaryController.getRightTriggerAxis().getButton(0.5).whenReleased(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
        primaryController.getRightTriggerAxis().getButton(0.1).whenPressed(
            new InstantCommand(() -> drivetrain.setTurbo(true))
        );
        primaryController.getRightTriggerAxis().getButton(0.1).whenReleased(
            new InstantCommand(() -> drivetrain.setTurbo(false))
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

        secondaryController.getBButton().onTrue(
            new setArmSafe(armExtender, armRotator, 22, 4.0)
        );

        secondaryController.getAButton().onTrue(
            new setArmSafe(armExtender, armRotator, 0, 0)
        );

        secondaryController.getXButton().onTrue(
            new setArmSafe(armExtender, armRotator, -87, 0)    
        );

        secondaryController.getYButton().onTrue(
            new setArmSafe(armExtender, armRotator, -105, 12)
        );

        secondaryController.getRightBumperButton().whenReleased(
            new setArmSafe(armExtender, armRotator,0.0, 0.0)
        );

        secondaryController.getRightTriggerAxis().getButton(0.1).whenReleased(
            new setArmSafe(armExtender, armRotator,0.0, 0.0)
        );

        secondaryController.getRightTriggerAxis().getButton(0.1).whenPressed(
            new setArmSafe(armExtender, armRotator,22.0, 4.0)
        );

        secondaryController.getRightBumperButton().whenPressed(
            new setArmSafe(armExtender, armRotator,22.0, 4.0)
        );

        
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
