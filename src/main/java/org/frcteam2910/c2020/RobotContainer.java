package org.frcteam2910.c2020;

import org.frcteam2910.c2020.commands.ArmRotationControlJoysticks;
import org.frcteam2910.c2020.commands.ChangeDriveMode;
import org.frcteam2910.c2020.commands.DriveBalanceCommand;
import org.frcteam2910.c2020.commands.VariableIntakeRPMCommand;
import org.frcteam2910.c2020.commands.ZeroAllWheels;
import org.frcteam2910.c2020.commands.ZeroGyroscope;
import org.frcteam2910.c2020.subsystems.ArmExtender;
import org.frcteam2910.c2020.subsystems.ArmRotator;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.AutoFormatShuffleboardTab;
import org.frcteam2910.c2020.util.GyroAutoChooser;
import org.frcteam2910.c2020.util.SideChooser;
import org.frcteam2910.common.robot.input.DPadButton;
import org.frcteam2910.common.robot.input.XboxController;

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

    private final AutoFormatShuffleboardTab driverReadoutTab;
    private final AutoFormatShuffleboardTab drivetrainTab;

    public RobotContainer() {
        instance = this;
        sideChooser = new SideChooser();
        gyroAutoChooser = new GyroAutoChooser();

        autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, sideChooser.getSide());
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);

        drivetrain.setController(primaryController);
        //intake.setController(secondaryController);

        driverReadoutTab = new AutoFormatShuffleboardTab(this, "Driver Readout", 2, 9);
        
        driverReadoutTab.addButtonToGrid("Autonomous Mode", getAutonomousChooser().getSendableChooser(), 2, 1);
        driverReadoutTab.addButtonToGrid("Zero Gyroscope", new ZeroGyroscope(getDrivetrainSubsystem()), 2, 1, 2);
        driverReadoutTab.addButtonToGrid("Side", getSideChooser().getSendableChooser(), 2, 1);
        driverReadoutTab.addButtonToGrid("Gyro Auto Correct", getGyroAutoAdjustMode().getSendableChooser(), 1, 1);
        driverReadoutTab.addButtonToGrid("Zero Wheels", new ZeroAllWheels(getDrivetrainSubsystem()), 1, 1);

        drivetrainTab = new AutoFormatShuffleboardTab(this, "Drivetrain");
        
        CommandScheduler.getInstance().setDefaultCommand(intake, new VariableIntakeRPMCommand(intake));
        CommandScheduler.getInstance().setDefaultCommand(armRotator, new ArmRotationControlJoysticks(armRotator));
        //CommandScheduler.getInstance().setDefaultCommand(armExtender, new ArmTranslationalControlJoysticks(armExtender, getArmTranslationalAxis()));

        configureButtonBindings();
    }

    public void updateTrajectoriesBasedOnSide(){
        autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, sideChooser.getSide());
        autonomousChooser.updateTrajectories(autonomousTrajectories);
    }

    public void updateSide(){
        drivetrain.setSide(sideChooser.getSide());
    }

    private void configureButtonBindings() {
        
        primaryController.getStartButton().onTrue(
                new InstantCommand(()->drivetrain.resetSteerAbsoluteAngle())
        );
        primaryController.getBackButton().onTrue(
                new ZeroGyroscope(drivetrain)
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
        primaryController.getRightTriggerAxis().getButton(0.5).onTrue(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.BALL_TRACK)
        );
        primaryController.getRightTriggerAxis().getButton(0.5).onFalse(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );

        primaryController.getAButton().onTrue(
            //new DriveBalanceCommand(drivetrain, false,true)
            new DriveBalanceCommand(drivetrain, false, true)
        );

        // primaryController.getXButton().whenPressed(
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
            
        // SmartDashboard.putData("Turn to Goal", new InstantCommand(() -> drivetrain.setTurnToTarget()));

        // SmartDashboard.putData("Limelight broken", new InstantCommand(() -> drivetrain.setLimelightOverride(true)));
        // SmartDashboard.putData("Limelight working", new InstantCommand(() -> drivetrain.setLimelightOverride(false)));
        // SmartDashboard.putData("set arm to 30 degrees", new InstantCommand(() -> armRotator.setArmDegreesPositionAbsolute(30)));
        // SmartDashboard.putData("set arm to 60 degrees", new InstantCommand(() -> armRotator.setArmDegreesPositionAbsolute(60)));
        // SmartDashboard.putData("set arm to 90 degrees", new InstantCommand(() -> armRotator.setArmDegreesPositionAbsolute(90)));
        // SmartDashboard.putData("set arm to 0 degrees", new InstantCommand(() -> armRotator.setArmDegreesPositionAbsolute(0)));
    }
    
    public Command getSelectedAutonomousCommand() {
        return autonomousChooser.getCommandBasedSelection(this);
    }
    
    public XboxController getPrimaryController() {
        return primaryController;
    }

    public XboxController getSecondaryController() {
        return secondaryController;
    }

    //#region Subsystems
    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrain;
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
    //#endregion

    //#region Shuffleboard - Tabs and Choosers

    public AutoFormatShuffleboardTab getDriverReadoutTab() {
        return driverReadoutTab;
    }

    public AutoFormatShuffleboardTab getDrivetrainTab() {
        return drivetrainTab;
    }
    
    public GyroAutoChooser getGyroAutoAdjustMode() {
        return gyroAutoChooser;
    }

    public SideChooser getSideChooser(){
        return sideChooser;
    }

    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }
    
    //#endregion

    public static RobotContainer getInstance() {
        if(instance == null) {
            instance = new RobotContainer();
            //throw new RuntimeException("Somehow, RobotContainer constructor was never called and/or never set the RobotContainer instance");
        }
        return instance;
    }
}
