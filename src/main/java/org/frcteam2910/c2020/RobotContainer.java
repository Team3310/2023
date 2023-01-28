package org.frcteam2910.c2020;

import java.io.IOException;

import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.DriverReadout;
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

    private AutonomousTrajectories autonomousTrajectories;
    private final AutonomousChooser autonomousChooser;
    private final SideChooser sideChooser;

    private final DriverReadout driverReadout;

    public RobotContainer() {
        sideChooser = new SideChooser();
        try {
            autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, sideChooser.getSide()==SideChooser.sideMode.BLUE?true:false);
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("Error building trajectories");
        }
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);

        drivetrain.setController(primaryController);

        driverReadout = new DriverReadout(this);

        CommandScheduler.getInstance().registerSubsystem(drivetrain);

        configureButtonBindings();
        
        instance = this;
    }

    private void configureButtonBindings() {

        // primaryController.getBackButton().whenPressed(
        //         new ZeroAll(balanceElevator, climbElevator, drivetrain)
        // );
        primaryController.getStartButton().whenPressed(
                new InstantCommand(()->drivetrain.resetSteerAbsoluteAngle())
        );
        primaryController.getRightBumperButton().whenPressed(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.ROBOT_CENTRIC)
        );
        primaryController.getRightBumperButton().whenReleased(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
//        primaryController.getLeftBumperButton().whenPressed(
//                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.LIMELIGHT)
//        );
        primaryController.getLeftBumperButton().whenReleased(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );

        primaryController.getXButton().whenReleased(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
        primaryController.getRightTriggerAxis().getButton(0.5).whenPressed(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.BALL_TRACK)
        );
        primaryController.getRightTriggerAxis().getButton(0.5).whenReleased(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );

        //Intake
        // secondaryController.getRightTriggerAxis().getButton(0.5).whenPressed(
        //         new IndexerBallStop(indexer)
        // );
        // secondaryController.getRightTriggerAxis().getButton(0.5).whenReleased(
        //         new IndexerSetSpeed(indexer, 0)
        // );
        // secondaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(
        //         new IntakeLiftSetAngle(intake, Constants.LIFT_MIN_ANGLE_DEGREES)
        // );
        // secondaryController.getDPadButton(DPadButton.Direction.DOWN).whenPressed(
        //         new IntakeLiftSetAngle(intake, Constants.LIFT_MAX_ANGLE_DEGREES)
        // );

        //Climb
        // secondaryController.getBackButton().whenPressed(
        //         new ClimbElevatorAutoZero(climbElevator)
        // );

        // //Indexer
        // secondaryController.getRightBumperButton().whenPressed(
        //         new FeedBalls(intake, indexer, drivetrain, shooter, Constants.INDEXER_RPM)
        // );
        // secondaryController.getRightBumperButton().whenReleased(
        //         new IntakeIndexerHaltTeleOp(intake, indexer, shooter, drivetrain, led)
        // );
        // secondaryController.getLeftBumperButton().whenPressed(
        //         new EjectBalls(intake, indexer, shooter)
        // );
        // secondaryController.getLeftBumperButton().whenReleased(
        //         new IntakeIndexerHaltTeleOp(intake, indexer, shooter, drivetrain, led)
        // );



        //Shooter

        // This is X on PS4!
        // secondaryController.getAButton().whenPressed(
        //         new ShooterShootWithHood(shooter, drivetrain, 1780, 11.0) //Fender
        // );

        // // This is CIRCLE on PS4!
        // secondaryController.getBButton().whenPressed(
        //         new ShooterShootWithHood(shooter, drivetrain, 2030, 30.7) //RT Wall 24
        // );

        // // This is TRIANGLE on PS4!
        // secondaryController.getYButton().whenPressed(
        //         new ShooterShootWithHood(shooter, DrivetrainSubsystem.SwervePivotPoint.BACK, drivetrain, 2230, 36) //Hangar Shot
        // );

        // This is SQUARE on PS4!
        // secondaryController.getXButton().whenPressed(
        //         new ShooterShootAllField(shooter, drivetrain)
        // );

        // secondaryController.getStartButton().whenPressed(
        //         new HoodAutoZero(shooter)
        // );


        //Misc
        secondaryController.getDPadButton(DPadButton.Direction.RIGHT).whenPressed(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
        // secondaryController.getDPadButton(DPadButton.Direction.LEFT).whenPressed(
        //         new ExperimentalEjectBalls(intake, indexer)
        // );
        secondaryController.getLeftJoystickButton().whenPressed(
                new InstantCommand(()-> drivetrain.setLimelightOverride(true))
        );
        secondaryController.getRightJoystickButton().whenPressed(
                new InstantCommand(()-> drivetrain.setLimelightOverride(false))
        );


        //     SmartDashboard.putData("Auto Zero Hood", new HoodAutoZero(shooter));
        //     SmartDashboard.putData("Auto Zero Climb", new ClimbElevatorAutoZero(climbElevator));
        //     SmartDashboard.putData("Set Intake speed 0", new IntakeSetSpeed(intake, 0.0));
        //     SmartDashboard.putData("Set Intake speed 1", new IntakeSetSpeed(intake, 1.0));
        //     SmartDashboard.putData("Set Indexer speed 0", new IndexerSetSpeed(indexer, 0.0));


        //     SmartDashboard.putData("Set Shooter speed 0", new ShooterSetSpeed(shooter, 0.0));
        //     SmartDashboard.putData("Set Shooter RPM 2055", new ShooterSetRPM(shooter, 2055));


        //     SmartDashboard.putData("Set Hood 32", new HoodSetAngle(shooter, 32.0));
        //     SmartDashboard.putData("Set Hood 45", new HoodSetAngle(shooter, 45.0));
        //     SmartDashboard.putData("Set Hood 42", new HoodSetAngle(shooter, 42.0));
        //     SmartDashboard.putData("Set Hood 38", new HoodSetAngle(shooter, 38.0));

        //     SmartDashboard.putData("Set Lift 0", new IntakeLiftSetAngle(intake, Constants.LIFT_MIN_ANGLE_DEGREES));
        //     SmartDashboard.putData("Set Lift 15", new IntakeLiftSetAngle(intake, 15.0));
        //     SmartDashboard.putData("Set Lift 35", new IntakeLiftSetAngle(intake, Constants.LIFT_MAX_ANGLE_DEGREES));
        //     SmartDashboard.putData("Reset Lift", new InstantCommand(() -> intake.resetLiftHomePosition()));

            SmartDashboard.putData("Turn to Goal", new InstantCommand(() -> drivetrain.setTurnToTarget()));


        // SmartDashboard.putData("Distance offset -20", new InstantCommand(()-> shooter.setShooterDistanceOffset(-20)));
        // SmartDashboard.putData("Distance offset -10", new InstantCommand(()-> shooter.setShooterDistanceOffset(-10)));
        // SmartDashboard.putData("Distance offset -5", new InstantCommand(()-> shooter.setShooterDistanceOffset(-5)));
        // SmartDashboard.putData("Distance offset 0", new InstantCommand(()-> shooter.setShooterDistanceOffset(0)));
        // SmartDashboard.putData("Distance offset +5", new InstantCommand(()-> shooter.setShooterDistanceOffset(5)));
        // SmartDashboard.putData("Distance offset +10", new InstantCommand(()-> shooter.setShooterDistanceOffset(10)));
        // SmartDashboard.putData("Distance offset +20", new InstantCommand(()-> shooter.setShooterDistanceOffset(20)));

        // SmartDashboard.putData("Set Shooter speed 0", new ShooterSetSpeed(shooter, 0.0));

        SmartDashboard.putData("Limelight broken", new InstantCommand(()-> drivetrain.setLimelightOverride(true)));
        SmartDashboard.putData("Limelight working", new InstantCommand(()-> drivetrain.setLimelightOverride(false)));

    }
    public Command getAutonomousCommand() {
        return autonomousChooser.getCommand(this);
    }

    private Axis getClimbElevatorAxis() {
        return secondaryController.getLeftYAxis();
    }

    private Axis getBalanceElevatorAxis(){
        return secondaryController.getRightYAxis();
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrain;
    }

    public SideChooser getSideChooser(){
        return sideChooser;
    }

//     public ClimbElevator getClimbElevator() {
//         return climbElevator;
//     }

//     public Intake getIntakeSubsystem() {
//         return intake;
//     }

//     public Shooter getShooter() {
//         return shooter;
//     }
//     public Indexer getIndexer() {
//         return indexer;
//     }
//     public LED getLED(){
//             return led;
//     }

//     public BalanceElevator getBalanceElevator(){
//         return balanceElevator;
//     }

    public XboxController getPrimaryController() {
        return primaryController;
    }

    public AutonomousChooser getAutonomousChooser() {
        return autonomousChooser;
    }

    public DriverReadout getDriverReadout() {
        return driverReadout;
    }

    public static RobotContainer getInstance() {
        if(instance == null) {
            //throw new RuntimeException("Somehow, RobotContainer constructor was never called and/or never set the RobotContainer instance");
        }
        return instance;
    }
}
