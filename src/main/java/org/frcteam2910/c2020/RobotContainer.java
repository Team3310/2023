package org.frcteam2910.c2020;

import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.*;
import org.frcteam2910.common.robot.input.*;
import org.frcteam2910.common.robot.input.DPadButton.Direction;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class RobotContainer {
    private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
    private final XboxController secondaryController = new XboxController(Constants.SECONDARY_CONTROLLER_PORT);

    private static RobotContainer instance;

    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Arm arm = Arm.getInstance();

    
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
        
        CommandScheduler.getInstance().setDefaultCommand(arm, new ArmJoystickControl(arm, getArmExtenderAxis(), getArmRotationAxis()));
        
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
        
        primaryController.getStartButton().onTrue(
                new InstantCommand(()->drivetrain.resetSteerAbsoluteAngle())
        );
        primaryController.getBackButton().onTrue(
                new ZeroAll(drivetrain)
        );
        primaryController.getRightBumperButton().onTrue(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.ROBOT_CENTRIC)
        );
        primaryController.getRightBumperButton().onFalse(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
        primaryController.getLeftBumperButton().onTrue(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.LIMELIGHT)
        );
        primaryController.getLeftBumperButton().onFalse(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
        primaryController.getYButton().onTrue(
                new InstantCommand(()->intake.setServoPosition(-1))
        );
        primaryController.getBButton().onTrue(
                new InstantCommand(()->intake.setServoPosition(1))
        );
        primaryController.getRightTriggerAxis().getButton(0.5).onFalse(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
        primaryController.getRightTriggerAxis().getButton(0.1).onTrue(
            new InstantCommand(() -> drivetrain.setTurbo(true))
        );
        primaryController.getRightTriggerAxis().getButton(0.1).onFalse(
            new InstantCommand(() -> drivetrain.setTurbo(false))
        );

        primaryController.getAButton().onTrue(
            //new DriveBalanceCommand(drivetrain, false,true)
            new DriveBalanceCommand(drivetrain, false, true)
        );

        // primaryController.getXButton().onTrue(
        //     new InstantCommand(()->drivetrain.setDriveControlMode(DriveControlMode.LIMELIGHT))
        // );

        //Misc
        secondaryController.getDPadButton(DPadButton.Direction.RIGHT).onTrue(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
        secondaryController.getLeftJoystickButton().onTrue(
                new InstantCommand(()-> drivetrain.setLimelightOverride(true))
        );
        secondaryController.getRightJoystickButton().onTrue(
                new InstantCommand(()-> drivetrain.setLimelightOverride(false))
        );
        secondaryController.getDPadButton(Direction.UP).onTrue(
                new InstantCommand(()-> arm.setTargetArmInchesPositionAbsolute(arm.getArmInches()+0.25))
        );
        secondaryController.getDPadButton(Direction.DOWN).onTrue(
                new InstantCommand(()-> arm.setTargetArmInchesPositionAbsolute(arm.getArmInches()-0.25))
        );
        secondaryController.getDPadButton(Direction.UP).onTrue(
                new InstantCommand(()-> arm.setTargetArmInchesPositionAbsolute(arm.getArmInches()+0.25))
        );

        secondaryController.getBButton().onTrue(
            new setArmSafe(arm, ScoreMode.INTAKE)
        );

        secondaryController.getAButton().onTrue(
            new setArmSafe(arm, ScoreMode.ZERO)
        );

        secondaryController.getXButton().onTrue(
            new setArmSafe(arm, ScoreMode.MID)    
        );

        secondaryController.getYButton().onTrue(
            new setArmSafe(arm, ScoreMode.HIGH)
        );

        secondaryController.getDPadButton(Direction.UP).onTrue(
            new InstantCommand(() -> arm.setArmDegreesZero(0))  
        );

        secondaryController.getRightTriggerAxis().getButton(0.1).onTrue(
            new InstantCommand(() -> System.out.println("ran cube intake command"))
        ).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> intake.setServoPosition(-1.0)),
                new setArmSafe(arm, ScoreMode.INTAKE)
            )    
        );

        // secondaryController.getRightTriggerAxis().getButton(0.1).onFalse(
        //     new SequentialCommandGroup(
        //     new setArmSafe(arm, ScoreMode.ZERO),
        //     new InstantCommand(() -> intake.setServoPosition(1.0))
        //    )    
        // );

        secondaryController.getRightBumperButton().onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> intake.setServoPosition(-1.0)),
                new setArmSafe(arm, ScoreMode.INTAKE)
            )    
        );

        secondaryController.getRightBumperButton().onFalse(
            new SequentialCommandGroup(
                new setArmSafe(arm, ScoreMode.ZERO),
                new InstantCommand(() -> intake.setServoPosition(1.0))
            )    
        );

        
        secondaryController.getStartButton().onTrue(new ArmExtenderZero(arm));
        secondaryController.getBackButton().onTrue(new InstantCommand(()->arm.setArmDegreesZero(0)));
            
        // SmartDashboard.putData("Turn to Goal", new InstantCommand(() -> drivetrain.setTurnToTarget()));

        // SmartDashboard.putData("Limelight broken", new InstantCommand(() -> drivetrain.setLimelightOverride(true)));
        // SmartDashboard.putData("Limelight working", new InstantCommand(() -> drivetrain.setLimelightOverride(false)));
        // SmartDashboard.putData("set arm to 30 degrees", new InstantCommand(() -> arm.setArmDegreesPositionAbsolute(30)));
        // SmartDashboard.putData("set arm to 60 degrees", new InstantCommand(() -> arm.setArmDegreesPositionAbsolute(60)));
        // SmartDashboard.putData("set arm to 90 degrees", new InstantCommand(() -> arm.setArmDegreesPositionAbsolute(90)));
        // SmartDashboard.putData("set arm to 0 degrees", new InstantCommand(() -> arm.setArmDegreesPositionAbsolute(0)));
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
    private Axis getArmExtenderAxis() {
        return secondaryController.getRightYAxis();
    }
    public Axis getIntakeAxis() {
        return secondaryController.getRightTriggerAxis();
    }
    public Axis getOuttakeAxis() {
        return secondaryController.getLeftTriggerAxis();
    }

    public void runCommand(Command command){
        command.schedule();
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

    public Arm getArm(){
        return arm;
    }

    public static RobotContainer getInstance() {
        if(instance == null) {
            //throw new RuntimeException("Somehow, RobotContainer constructor was never called and/or never set the RobotContainer instance");
        }
        return instance;
    }
}
