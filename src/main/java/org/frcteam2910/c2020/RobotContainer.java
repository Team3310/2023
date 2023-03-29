package org.frcteam2910.c2020;

import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.commands.auton.OnToBridge;
import org.frcteam2910.c2020.commands.auton.OneObjectMid;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.LimelightMode;
import org.frcteam2910.c2020.util.*;
import org.frcteam2910.common.robot.input.*;
import org.frcteam2910.common.robot.input.DPadButton.Direction;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


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

        DrivetrainSubsystem.getInstance().setController(primaryController);

        driverReadout = new DriverReadout(this);

        CommandScheduler.getInstance().registerSubsystem(DrivetrainSubsystem.getInstance());
        CommandScheduler.getInstance().registerSubsystem(Intake.getInstance());
        
        CommandScheduler.getInstance().setDefaultCommand(Arm.getInstance(), new ArmJoystickControl(Arm.getInstance(), getArmExtenderAxis(), getArmRotationAxis()));
        
        configureButtonBindings();
        
        instance = this;
    }

    public void recreateTrajectoriesBasedOnSide(){
        autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, sideChooser.getSide());
        autonomousChooser.updateTrajectories(autonomousTrajectories);
        // SmartDashboard.putString("Side", sideChooser.getSide().toString());
    }

    public void updateSide(){
        DrivetrainSubsystem.getInstance().setSide(sideChooser.getSide());
    }

    private void configureButtonBindings() {
        //#region Primary/Driver Controller
        primaryController.getStartButton().onTrue(
                new InstantCommand(()->DrivetrainSubsystem.getInstance().resetSteerAbsoluteAngle())
        );
        primaryController.getBackButton().onTrue(
                new ZeroAll(DrivetrainSubsystem.getInstance())
        );

        primaryController.getBButton().onTrue(
                new InstantCommand(()->Intake.getInstance().setServoPosition(1))
        );
        primaryController.getAButton().onTrue(
            new SequentialCommandGroup(
                new ChangeDriveMode(drivetrain, DriveControlMode.BRIDGE_VOLTAGE),
                new InstantCommand(()->drivetrain.setBridgeDriveVoltage(-5)),
                new WaitUntilCommand(() -> drivetrain.getRollDegreesOffLevel()>20),
                new WaitUntilCommand(() -> drivetrain.getRollDegreesOffLevel()<12.25),
                new DriveBalanceCommand(drivetrain, true, false, false),
                new DriveBalanceCommand(drivetrain, true, false, true)
            )
        );
        primaryController.getYButton().onTrue(
                new InstantCommand(()->Intake.getInstance().setServoPosition(-1))
        );
        primaryController.getXButton().onTrue(
            new InstantCommand(() -> DrivetrainSubsystem.getInstance().zeroGyro())
            // new ChangeDriveMode(drivetrain, DriveControlMode.HOLD)
        );


        
        primaryController.getLeftBumperButton().onTrue(
                new ChangeDriveMode(DrivetrainSubsystem.getInstance(), DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
        primaryController.getLeftBumperButton().onFalse(
                new ChangeDriveMode(DrivetrainSubsystem.getInstance(), DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );

        primaryController.getRightBumperButton().onTrue(
            new InstantCommand(() -> {
                DrivetrainSubsystem.getInstance().setTurbo(true);
                DrivetrainSubsystem.getInstance().setDriveControlMode(DriveControlMode.JOYSTICKS);
            })
        );
        primaryController.getRightBumperButton().onFalse(
            new InstantCommand(() -> DrivetrainSubsystem.getInstance().setTurbo(false))
        );


        primaryController.getRightTriggerAxis().onTrue(
            new InstantCommand(() -> {
                DrivetrainSubsystem.getInstance().setDriveControlMode(DriveControlMode.LIMELIGHT);
                DrivetrainSubsystem.getInstance().setLimelightMode(LimelightMode.RETROREFLECTIVE);
            })
        );
        primaryController.getRightTriggerAxis().onFalse(
            new InstantCommand(() -> {
                DrivetrainSubsystem.getInstance().setDriveControlMode(DriveControlMode.JOYSTICKS);
                DrivetrainSubsystem.getInstance().setLimelightMode(LimelightMode.NONE);
            })
        );

        primaryController.getLeftBumperButton().onTrue(
            new InstantCommand(()->{
                intake.stopRollingOnTriggeredCubeIntakeDIO = true;
                intake.setCubeIntakeDeployTargetPosition(110);
                intake.setCubeRollerRPM(2000);
            })
        );
        primaryController.getLeftBumperButton().onFalse(
            new InstantCommand(()->{
                intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                intake.setCubeIntakeDeployTargetPosition(0);
                intake.setCubeRollerRPM(0);
            })
        );

        primaryController.getLeftTriggerAxis().onTrue(
            new InstantCommand(()->{
                intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                intake.setCubeRollerRPM(-2000);
            })
        );
        primaryController.getLeftTriggerAxis().onFalse(
            new InstantCommand(()->{
                intake.setCubeRollerRPM(0);
            })
        );
        //#endregion

        //#region Second/Operator Controller
        // secondaryController.getDPadButton(DPadButton.Direction.RIGHT).onTrue(
        //         new ChangeDriveMode(DrivetrainSubsystem.getInstance(), DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        //);
        // secondaryController.getLeftJoystickButton().onTrue(
        //         new InstantCommand(()-> DrivetrainSubsystem.getInstance().setLimelightOverride(true))
        // );
        // secondaryController.getRightJoystickButton().onTrue(
        //         new InstantCommand(()-> DrivetrainSubsystem.getInstance().setLimelightOverride(false))
        // );
        secondaryController.getDPadButton(Direction.UP).onTrue(
                new InstantCommand(()-> Arm.getInstance().setTargetArmInchesPositionAbsolute(Arm.getInstance().getArmInches()+0.25))
        );
        secondaryController.getDPadButton(Direction.DOWN).onTrue(
                new InstantCommand(()-> Arm.getInstance().setTargetArmInchesPositionAbsolute(Arm.getInstance().getArmInches()-0.25))
        );
        secondaryController.getDPadButton(Direction.UP).onTrue(
                new InstantCommand(()-> Arm.getInstance().setTargetArmInchesPositionAbsolute(Arm.getInstance().getArmInches()+0.25))
        );

        secondaryController.getStartButton().onTrue(
            new ArmExtenderZero(Arm.getInstance())
        );
        secondaryController.getBackButton().onTrue(
            new InstantCommand(()->Arm.getInstance().setArmDegreesZero(0))
        );
        

        secondaryController.getBButton().onTrue(
            new SetArmSafely( ScoreMode.ZERO)
        );
        secondaryController.getAButton().onTrue(
            new SetArmSafely( ScoreMode.LOW)
        );
        secondaryController.getXButton().onTrue(
            new SetArmSafely( ScoreMode.MID)
        );
        secondaryController.getYButton().onTrue(
            new SetArmSafely( ScoreMode.HIGH)
        );


        // Cone Intake
        secondaryController.getRightBumperButton().onTrue(
            new SequentialCommandGroup(
                new SetServosOut(Intake.getInstance()),
                new SetIntakeRPM(Intake.getInstance(), Constants.ARM_INTAKE_COLLECT_RPM),
                new SetArmSafely(ScoreMode.CONE_INTAKE)
            )    
        );
        secondaryController.getRightBumperButton().onFalse(
            // If we grabbed a cone, we want to continue intaking until we're back at ZERO
            new SequentialCommandGroup(
                new SetArmSafely(true, true),
                new SetIntakeRPM(Intake.getInstance(), 0),
                new SetServosIn(Intake.getInstance())
            )    
        );

        // Cube Intake
        secondaryController.getRightTriggerAxis().onTrue(
            new SequentialCommandGroup(
                new SetIntakeRPM(Intake.getInstance(), Constants.ARM_CUBE_INTAKE_COLLECT_RPM),
                new SetArmSafely(ScoreMode.CUBE_INTAKE, false, false),
                new InstantCommand(() -> {
                    // This flag is set so that we can force stuff to stop moving once cone/cube is in arm intake
                    Intake.getInstance().stopRollingOnTriggeredCubeIntakeDIO = false;
                    Intake.getInstance().stopRollingOnTriggeredArmIntakeDIO = true;
                    intake.resetIntakeDIOTimestamp();
                })
                
                
                // For now, it's up to the operator to stop intaking to not pop the cube
                
                // Hasn't been tested, but an idea: need to stop intaking when intake.getCubeSensor().get() is true
                // new SequentialCommandGroup(
                //     new ParallelRaceGroup(
                //         new WaitCommand(5.0), // Ya got 5 seconds to intake that cube, man.
                //         new WaitUntilCommand(() -> intake.getCubeSensor().get())
                //     ),
                //     new InstantCommand(()->intake.setIntakeHold()),
                // )
            )    
        );
        secondaryController.getRightTriggerAxis().onFalse(
            // If we grabbed a cube, we want to continue intaking until we're back at ZERO
            // CommandScheduler.getInstance().clearButtons();
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // This flag is set so that we can force stuff to stop moving once cone/cube is in arm intake
                    Intake.getInstance().stopRollingOnTriggeredCubeIntakeDIO = false;
                    Intake.getInstance().stopRollingOnTriggeredArmIntakeDIO = false;
                    intake.resetIntakeDIOTimestamp();
                }),
                new InstantCommand(()->intake.setArmIntakeHold()),
                new SetArmSafely(true, false)
           )
        );
        
        // Outtake Cone
        secondaryController.getLeftBumperButton().onTrue(
            new SetIntakeRPM(Intake.getInstance(), Constants.ARM_INTAKE_SPIT_RPM)
        );
        secondaryController.getLeftBumperButton().onFalse(
            new PutIntakeZeroAfterOuttake(Intake.getInstance(), Arm.getInstance())
        );

        // Outtake Cube
        secondaryController.getLeftTriggerAxis().onTrue(
            new SequentialCommandGroup(
                new SetArmExtender(arm, 17.5),
                new InstantCommand(() -> {
                    Intake.getInstance().stopRollingOnTriggeredCubeIntakeDIO = false;
                    Intake.getInstance().stopRollingOnTriggeredArmIntakeDIO = false;
                }),
                new SetIntakeRPM(Intake.getInstance(), Constants.ARM_CUBE_INTAKE_SPIT_RPM)
            )
        );
        secondaryController.getLeftTriggerAxis().onFalse(
            new PutIntakeZeroAfterOuttake(Intake.getInstance(), Arm.getInstance())
        );

        //#endregion

        // SmartDashboard.putData("Turn to Goal", new InstantCommand(() -> DrivetrainSubsystem.getInstance().setTurnToTarget()));
        //SmartDashboard.putData("set drive control mode voltage", new InstantCommand(() -> {drivetrain.setBridgeDriveVoltage(1.0); drivetrain.setDriveControlMode(DriveControlMode.BRIDGE_VOLTAGE);}));
        SmartDashboard.putData("extendo zero", new ArmExtenderZero(Arm.getInstance()));
        // SmartDashboard.putData("roller rpm to 1000", new InstantCommand(() -> Intake.getInstance().setCubeRollerRPM(1000)));
        // SmartDashboard.putData("roller rpm to 2000", new InstantCommand(() -> Intake.getInstance().setCubeRollerRPM(2000)));
        // SmartDashboard.putData("roller rpm to 100", new InstantCommand(() -> Intake.getInstance().setCubeRollerRPM(100)));
        // SmartDashboard.putData("roller rpm to 0", new InstantCommand(() -> Intake.getInstance().setCubeRollerRPM(0)));
        SmartDashboard.putData("lift zero", new InstantCommand(() -> Intake.getInstance().setCubeIntakeDeployHome(0)));
        // SmartDashboard.putData("lift degrees to 30", new InstantCommand(() -> Intake.getInstance().setCubeIntakeDeployTargetPosition(30)));
        // SmartDashboard.putData("lift degrees to 70", new InstantCommand(() -> Intake.getInstance().setCubeIntakeDeployTargetPosition(70)));
        SmartDashboard.putData("lift degrees to 111", new InstantCommand(() -> Intake.getInstance().setCubeIntakeDeployTargetPosition(111)));
        SmartDashboard.putData("lift degrees to 0", new InstantCommand(() -> Intake.getInstance().setCubeIntakeDeployTargetPosition(0)));
        //SmartDashboard.putData("Zero Gyro", new InstantCommand(() -> DrivetrainSubsystem.getInstance().zeroGyro()));
        // SmartDashboard.putData("Limelight working", new InstantCommand(() -> DrivetrainSubsystem.getInstance().setLimelightOverride(false)));
        // SmartDashboard.putData("set Arm.getInstance() to 30 degrees", new InstantCommand(() -> Arm.getInstance().setArmDegreesPositionAbsolute(30)));
        // SmartDashboard.putData("set Arm.getInstance() to 60 degrees", new InstantCommand(() -> Arm.getInstance().setArmDegreesPositionAbsolute(60)));
        // SmartDashboard.putData("set Arm.getInstance() to 90 degrees", new InstantCommand(() -> Arm.getInstance().setArmDegreesPositionAbsolute(90)));
        // SmartDashboard.putData("set Arm.getInstance() to 0 degrees", new InstantCommand(() -> Arm.getInstance().setArmDegreesPositionAbsolute(0)));
    }
    public Command getAutonomousCommand() {
        return autonomousChooser.getCommand(instance);
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return DrivetrainSubsystem.getInstance();
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
    public Boolean getIntakeAxis() {
        return secondaryController.getRightTriggerAxis().getAsBoolean();
    }
    public Boolean getOuttakeAxis() {
        return secondaryController.getLeftTriggerAxis().getAsBoolean();
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
        return Intake.getInstance();
    }

    public Arm getArm(){
        return Arm.getInstance();
    }

    public static RobotContainer getInstance() {
        if(instance == null) {
            //throw new RuntimeException("Somehow, RobotContainer constructor was never called and/or never set the RobotContainer instance");
        }
        return instance;
    }
}
