package org.frcteam2910.c2020;

import java.io.IOException;

import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.DriverReadout;
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

    private final DriverReadout driverReadout;

    public RobotContainer() {
        try {
            autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
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

        primaryController.getStartButton().onTrue(
                new InstantCommand(()->drivetrain.resetSteerAbsoluteAngle())
        );
        primaryController.getRightBumperButton().onTrue(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.ROBOT_CENTRIC)
        );
        primaryController.getRightBumperButton().onFalse(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
//        primaryController.getLeftBumperButton().onTrue(
//                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.LIMELIGHT)
//        );
        primaryController.getLeftBumperButton().onFalse(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );

        primaryController.getXButton().onFalse(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );
        primaryController.getRightTriggerAxis().getButton(0.5).onTrue(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.BALL_TRACK)
        );
        primaryController.getRightTriggerAxis().getButton(0.5).onFalse(
                new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        );

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
    }
    public Command getAutonomousCommand() {
        return autonomousChooser.getCommand(this);
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrain;
    }

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
