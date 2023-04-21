package org.frcteam2910.c2020;

import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.LimelightMode;
import org.frcteam2910.c2020.subsystems.Intake.IntakeStopMode;
import org.frcteam2910.c2020.util.*;
import org.frcteam2910.common.robot.input.*;
import org.frcteam2910.common.robot.input.DPadButton.Direction;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        instance = this;
        sideChooser = new SideChooser();
        gyroAutoChooser = new GyroAutoChooser();

        autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, sideChooser.getSide());
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);

        drivetrain.setController(primaryController);

        driverReadout = new DriverReadout(this);

        CommandScheduler.getInstance().registerSubsystem(drivetrain);
        CommandScheduler.getInstance().registerSubsystem(intake);
        CommandScheduler.getInstance().setDefaultCommand(arm, new ArmJoystickControl(arm, getArmExtenderAxis(), getArmRotationAxis()));
        
        configureButtonBindings();
    }

    public AutonomousTrajectories getTrajectories(){
        return autonomousTrajectories;
    }

    public void recreateTrajectoriesBasedOnSide(){
        autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, sideChooser.getSide());
        autonomousChooser.updateTrajectories(autonomousTrajectories);
        // SmartDashboard.putString("Side", sideChooser.getSide().toString());
    }

    private void configureButtonBindings() {
        //#region Primary/Driver Controller
        primaryController.getStartButton()
            .onTrue(
                new InstantCommand(()->drivetrain.resetSteerAbsoluteAngle())
            );
        primaryController.getBackButton()
            .onTrue(
                new ZeroAll(drivetrain)
            );

        primaryController.getBButton()
            .onTrue(
                new InstantCommand(()->intake.setServoPosition(1))
            );
        // primaryController.getAButton()
        //     .onTrue(
        //         new SequentialCommandGroup(
        //             new DriveBalanceCommand(drivetrain, false),
        //             new DriveBalanceCommand(drivetrain, true)
        //         )
        //     );
        primaryController.getYButton()
            .onTrue(
                new InstantCommand(()->intake.setServoPosition(-1))
            );
        primaryController.getXButton()
            .onTrue(
                // new InstantCommand(() -> drivetrain.zeroGyro())
                new ChangeDriveMode(drivetrain, DriveControlMode.HOLD)
            );

        // Cube intake
        primaryController.getLeftBumperButton()
        .onTrue(
            new SequentialCommandGroup(
                new SetArmSafely(ScoreMode.CUBE_INTAKE),
                new InstantCommand(()->{
                    intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                    intake.stopRollingOnTriggeredArmIntakeDIO = true;
                    // intake.setCubeIntakeDeployTargetPosition(110);
                    intake.resetIntakeDIOTimestamp();
                    intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_HANDOFF_RPM * 2.0, true);
                }),
                new SetArmIntakeRPM(intake, Constants.ARM_CUBE_INTAKE_COLLECT_RPM, true),
                new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES)
            )
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(()->intake.setCubeRollerRPM(0, true)),
                new SetArmSafely(true, false)
            )
        );

        // Cube outtake
        primaryController.getLeftTriggerAxis()
            .onTrue(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(()->{
                            intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                            intake.stopRollingOnTriggeredArmIntakeDIO = false;
                            intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_SPIT_RPM, true);
                        }),
                        new SequentialCommandGroup(
                            new WaitCommand(0.5),
                            new SetArmIntakeRPM(intake, Constants.CUBE_INTAKE_ROLLER_COLLECT_RPM, true)
                        )
                    )
                )
            )
            .onFalse(
                new ParallelCommandGroup(
                    new SetArmSafely(true, false),
                    new InstantCommand(()->{
                        intake.setCubeRollerRPM(0, true);
                    }),
                    new SetArmIntakeRPM(intake, 0, true)
                )
            );

        primaryController.getRightBumperButton()
            .onTrue(
                new ParallelCommandGroup(
                    new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS),
                    new InstantCommand(() -> drivetrain.setTurbo(true))
                ))
            .onFalse(
                new InstantCommand(() -> drivetrain.setTurbo(false))
            );

        //lock to 180 code
        // primaryController.getRightTriggerAxis()
        //     .onTrue(
        //         new ParallelCommandGroup(
        //             new InstantCommand(() -> drivetrain.setDriveBrake()),
        //             new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.LOCK_180)
        //         )
        //     )
        //     .onFalse(
        //         new ParallelCommandGroup(
        //             new InstantCommand(() -> drivetrain.setDriveCoast()),
        //             new InstantCommand(() -> drivetrain.setCommandedGyroAngle(drivetrain.getPose().rotation.toDegrees())),
        //             new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        //         )
        //     );

        primaryController.getRightTriggerAxis().onTrue(
            new CubeSpitSlow(intake)
        ).onFalse(
            new ParallelCommandGroup(
                new SetArmSafely(true, false),
                new InstantCommand(()->intake.setCubeRollerRPM(0, true)),
                new SetArmIntakeRPM(intake, 0, true)
            )
        );

        primaryController.getDPadButton(Direction.LEFT).onTrue(
            //new InstantCommand(()->intake.setCubeIntakeDeployZeroReference(0))
            new CubeIntakeLiftZero(intake)
        );


        // primaryController.getDPadButton(Direction.UP).onTrue(new FlashLEDs(LED.getInstance(), true));

        // primaryController.getDPadButton(Direction.UP).onTrue(new FlashLEDs(LED.getInstance(), false));

        // primaryController.getDPadButton(Direction.LEFT).onTrue(new InstantCommand(()->intake.setStopType(IntakeStopMode.POSITION)));

        // primaryController.getDPadButton(Direction.DOWN).onTrue(new InstantCommand(()->intake.setStopType(IntakeStopMode.TIME)));
        //#endregion

        //#region Second/Operator Controller
        secondaryController.getDPadButton(Direction.UP)
            .onTrue(
                new SetArmSafely(ScoreMode.CUBE_HIGH)
            );

        secondaryController.getDPadButton(Direction.LEFT)
            .onTrue(
                new SetArmSafely(ScoreMode.CUBE_MID)
            );

        secondaryController.getDPadButton(Direction.DOWN)
            .onTrue(
                new SetArmSafely(ScoreMode.CUBE_LOW)
            );

        secondaryController.getStartButton()
            .onTrue(
                new ArmExtenderZero(arm)
            );
        secondaryController.getBackButton()
            .onTrue(
                new InstantCommand(()->arm.setArmRotatorZeroReference(0))
            );

        secondaryController.getBButton()
            .onTrue(
                new SetArmSafely(ScoreMode.HOME)
            );
        secondaryController.getAButton()
            .onTrue(
                new SetArmSafely(ScoreMode.CONE_LOW)
            );
        secondaryController.getXButton()
            .onTrue(
                new SetArmSafely(ScoreMode.CONE_MID)
            );
        secondaryController.getYButton()
            .onTrue(
                new SetArmSafely(ScoreMode.CONE_HIGH)
            );


        // Cone Intake
        secondaryController.getRightBumperButton()
            .onTrue(
                new SequentialCommandGroup(
                    new SetServosOut(intake),
                    new SetArmIntakeRPM(intake, Constants.ARM_CONE_INTAKE_COLLECT_RPM, true),
                    new SetArmSafely(ScoreMode.CONE_INTAKE)
                )    
            ).onFalse(
            // If we grabbed a cone, we want to continue intaking until we're back at ZERO
            new SequentialCommandGroup(
                new SetArmSafely(true, true),
                new SetArmIntakeRPM(intake, 50, true),
                new SetServosIn(intake)
            )
            );

        // Cube Intake
        secondaryController.getRightTriggerAxis()
            .onTrue(
                new SequentialCommandGroup(
                    new SetArmIntakeRPM(intake, Constants.ARM_CUBE_INTAKE_COLLECT_RPM, true),
                    new SetArmSafely(ScoreMode.CUBE_INTAKE, false, false),
                    new InstantCommand(() -> {
                        // This flag is set so that we can force stuff to stop moving once cone/cube is in arm intake
                        intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                        intake.stopRollingOnTriggeredArmIntakeDIO = true;
                        intake.resetIntakeDIOTimestamp();
                    })
                )    
            )
            .onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // This flag is set so that we can force stuff to stop moving once cone/cube is in arm intake
                    intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                    intake.stopRollingOnTriggeredArmIntakeDIO = false;
                    intake.resetIntakeDIOTimestamp();
                }),
                new InstantCommand(()->intake.setArmIntakeRPM(0, true)),
                new SetArmSafely(true, false)
           )
        );
        
        // Outtake Cone
        secondaryController.getLeftBumperButton()
            .onTrue(
                new SetArmIntakeRPM(intake, Constants.ARM_CONE_INTAKE_SPIT_RPM, true)
            )
            .onFalse(
                new PutIntakeZeroAfterOuttake(intake, arm)
            );

        // Outtake Cube
        secondaryController.getLeftTriggerAxis()
            .onTrue(
                new SequentialCommandGroup(
                    // new CubeExtend(arm, true),
                    new InstantCommand(() -> {
                        intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                        intake.stopRollingOnTriggeredArmIntakeDIO = false;
                        intake.resetIntakeDIOTimestamp();
                    }),
                    new SetArmIntakeRPM(intake, Constants.ARM_CUBE_INTAKE_SPIT_RPM, true)
                ))
            .onFalse(
                new PutIntakeZeroAfterOuttake(intake, arm)
            );

        //#endregion

        // SmartDashboard.putData("Turn to Goal", new InstantCommand(() -> drivetrain.setTurnToTarget()));
        // SmartDashboard.putData("set drive control mode voltage", new InstantCommand(() -> {drivetrain.setBridgeDriveVoltage(1.0); drivetrain.setDriveControlMode(DriveControlMode.BRIDGE_VOLTAGE);}));
        // SmartDashboard.putData("extendo zero", new ArmExtenderZero(arm));
        // SmartDashboard.putData("LL CUBE_TRACK", new InstantCommand(() ->
        // {
        //     drivetrain.setLimelightMode(LimelightMode.AUTON_CUBE_TRACK);
        //     drivetrain.setDriveControlMode(DriveControlMode.LIMELIGHT_AUTON);
        // }));
        // SmartDashboard.putData("LL CUBE_INTAKE", new InstantCommand(() ->
        // {
        //     drivetrain.setLimelightMode(LimelightMode.CUBE_INTAKE);
        //     drivetrain.setDriveControlMode(DriveControlMode.LIMELIGHT);
        // }));
        // SmartDashboard.putData("LL NORMAL", new InstantCommand(() ->
        // {
        //     drivetrain.setLimelightMode(LimelightMode.NONE);
        //     drivetrain.setDriveControlMode(DriveControlMode.JOYSTICKS);
        // }));
        // SmartDashboard.putData("extendo zero", new ArmExtenderZero(arm));
        // SmartDashboard.putData("cancel all command", new InstantCommand(()->CommandScheduler.getInstance().cancelAll()));
        // SmartDashboard.putData("arm to zero", new SetArmSafely(ScoreMode.HOME));
        // SmartDashboard.putData("arm to high", new SetArmSafely(ScoreMode.CONE_HIGH));
        // SmartDashboard.putData("roller rpm to 1000", new InstantCommand(() -> intake.setCubeRollerRPM(1000)));
        // SmartDashboard.putData("roller rpm to 2000", new InstantCommand(() -> intake.setCubeRollerRPM(2000)));
        // SmartDashboard.putData("roller rpm to 100", new InstantCommand(() -> intake.setCubeRollerRPM(100)));
        // SmartDashboard.putData("roller rpm to 0", new InstantCommand(() -> intake.setCubeRollerRPM(0)));
        SmartDashboard.putData("lift zero", new InstantCommand(() -> intake.setCubeIntakeDeployZeroReference(0)));
        // SmartDashboard.putData("lift degrees to 30", new InstantCommand(() -> intake.setCubeIntakeDeployTargetPosition(30)));
        // SmartDashboard.putData("lift degrees to 70", new InstantCommand(() -> intake.setCubeIntakeDeployTargetPosition(70)));
        // SmartDashboard.putData("lift degrees to 111", new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES));
        // SmartDashboard.putData("lift degrees to 0", new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES));
        // SmartDashboard.putData("Zero Gyro", new InstantCommand(() -> drivetrain.zeroGyro()));
        // SmartDashboard.putData("Limelight working", new InstantCommand(() -> drivetrain.setLimelightOverride(false)));
        // SmartDashboard.putData("set arm to 30 degrees", new InstantCommand(() -> arm.setArmDegreesPositionAbsolute(30)));
        // SmartDashboard.putData("set arm to 60 degrees", new InstantCommand(() -> arm.setArmDegreesPositionAbsolute(60)));
        // SmartDashboard.putData("set arm to 90 degrees", new InstantCommand(() -> arm.setArmDegreesPositionAbsolute(90)));
        // SmartDashboard.putData("set arm to 0 degrees", new InstantCommand(() -> arm.setArmDegreesPositionAbsolute(0)));
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
    // public Boolean getOuttakeAxis() {
    //     return secondaryController.getLeftTriggerAxis().getAsBoolean();
    // }

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
