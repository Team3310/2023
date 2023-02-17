package org.frcteam2910.c2020;

import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.DriverReadout;
import org.frcteam2910.c2020.util.SideChooser;
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

        autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, sideChooser.getSide());
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);

        drivetrain.setController(primaryController);

        driverReadout = new DriverReadout(this);

        CommandScheduler.getInstance().registerSubsystem(drivetrain);

        configureButtonBindings();
        
        instance = this;
    }

    public void updateTrajectoriesBasedOnSide(){
        autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, sideChooser.getSide());
        autonomousChooser.updateTrajectories(autonomousTrajectories);
        SmartDashboard.putString("Side", sideChooser.getSide().toString());
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

        // primaryController.getXButton().whenReleased(
        //         new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        // );
        // primaryController.getRightTriggerAxis().getButton(0.5).whenPressed(
        //         new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.BALL_TRACK)
        // );
        // primaryController.getRightTriggerAxis().getButton(0.5).whenReleased(
        //         new ChangeDriveMode(drivetrain, DrivetrainSubsystem.DriveControlMode.JOYSTICKS)
        // );

        primaryController.getAButton().whenPressed(
            //new DriveBalanceCommand(drivetrain, false,true)
            new InstantCommand(()->drivetrain.setDriveControlMode(DriveControlMode.HOLD))
        );

        // primaryController.getXButton().whenPressed(
        //     new InstantCommand(()->drivetrain.setDriveControlMode(DriveControlMode.LIMELIGHT))
        // );
        primaryController.getXButton().whenPressed(
           new InstantCommand(()->drivetrain.setServosOut())
        );

        primaryController.getYButton().whenPressed(
           new InstantCommand(()->drivetrain.setServosIn())
        );

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
