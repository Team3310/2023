package org.frcteam2910.c2020.util;

import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.DriveBalanceCommand;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.commands.auton.*;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.RigidTransform2;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousChooser {
    private AutonomousTrajectories trajectories;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("Left Three Object Far Spit", AutonomousMode.THREE_OBJECT_FAR);
        // autonomousModeChooser.addOption("Left Two Object Close", AutonomousMode.THREE_OBJECT_CLOSE);
        // autonomousModeChooser.addOption("Left Two Object Close Balance", AutonomousMode.THREE_OBJECT_CLOSE_BALANCE);
        autonomousModeChooser.addOption("Right Three Object", AutonomousMode.THREE_OBJECT_BRIDGE);
        autonomousModeChooser.addOption("Right Two Object", AutonomousMode.TWO_OBJECT_BRIDGE);
        autonomousModeChooser.addOption("Right Two Object Balance", AutonomousMode.TWO_OBJECT_BRIDGE_BALANCE);
        autonomousModeChooser.addOption("Right Two Half Object Balance", AutonomousMode.TWO_HALF_OBJECT_BRIDGE_BALANCE);
        // autonomousModeChooser.addOption("On To Bridge", AutonomousMode.TO_BRIDGE);
        // autonomousModeChooser.addOption("Up Bridge", AutonomousMode.UP_BRIDGE);
        // autonomousModeChooser.addOption("Bridge Balance", AutonomousMode.BALANCE);
        // autonomousModeChooser.addOption("Right Two Object", AutonomousMode.RIGHT_TWO_OBJECT);
        // autonomousModeChooser.addOption("Two Object Bridge Balance", AutonomousMode.TWO_OBJECT_BALANCE);
        autonomousModeChooser.addOption("One Object Bridge Balance", AutonomousMode.ONE_OBJECT_BALANCE);
        autonomousModeChooser.addOption("One Half Object Bridge Balance", AutonomousMode.ONE_HALF_OBJECT_BALANCE);
        autonomousModeChooser.addOption("One Object Drive Away", AutonomousMode.ONE_OBJECT_DRIVE_AWAY);
  
        // autonomousModeChooser.addOption("7 Feet", AutonomousMode.SEVEN_FEET);
        // autonomousModeChooser.addOption("sCurve", AutonomousMode.S_CURVE);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }

    public void updateTrajectories(AutonomousTrajectories trajectories){
        this.trajectories=trajectories;
    }

    private Command getSevenFeet(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getSevenFeet());

        follow(command, container, trajectories.getSevenFeet());

        return command;
    }

    public Command get_sCurve(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.get_sCurve());

        follow(command, container, trajectories.get_sCurve());

        return command;
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case SEVEN_FEET:
                return getSevenFeet(container);
            case S_CURVE:
                return get_sCurve(container); 
            case THREE_OBJECT_FAR:
                return new LeftThreeObjectFarSpit(container, trajectories);
            case THREE_OBJECT_CLOSE:
                return new LeftTwoObjectClose(container, trajectories);
            case THREE_OBJECT_BRIDGE:
                return new RightSideThreeCone(container, trajectories); 
            case TWO_OBJECT_BRIDGE:
                return new RightSideTwoCone(container, trajectories);
            case TO_BRIDGE:
                return new OnToBridge(container, trajectories,-5);
            case BALANCE:
                return new DriveBalanceCommand(container.getDrivetrainSubsystem(), true, false);
            case THREE_OBJECT_CLOSE_BALANCE:
                return new LeftTwoObjectBalance(container, trajectories);
            case ONE_OBJECT_BALANCE:
                return new OneObjectMidBalance(container, trajectories);   
            case TWO_OBJECT_BALANCE:
                return new TwoObjectMidBalance(container, trajectories); 
            case RIGHT_TWO_OBJECT:
                return new RightSideTwoCone(container, trajectories);
            case ONE_OBJECT_DRIVE_AWAY:
                return new PlaceConeAndLeave(container, trajectories); 
            case TWO_OBJECT_BRIDGE_BALANCE:
                return new RightSideTwoConeBalance(container, trajectories);    
            case TWO_HALF_OBJECT_BRIDGE_BALANCE:
                return new RightSide2HalfBalance(container, trajectories);   
            case ONE_HALF_OBJECT_BALANCE:
                return new OneHalfObjectMidBalance(container, trajectories);     
            default:
                return getSevenFeet(container);
        }
        //return get10BallAutoCommand(container);
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory));
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(trajectory.calculate(0.0).getPathState().getRotation())));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(), trajectory.calculate(0.0).getPathState().getRotation()))));
    }

    public enum AutonomousMode { 
        SEVEN_FEET, 
        S_CURVE,
        THREE_OBJECT_FAR,
        THREE_OBJECT_CLOSE,
        THREE_OBJECT_BRIDGE,
        TWO_OBJECT_BRIDGE,
        TO_BRIDGE,
        UP_BRIDGE,
        BALANCE,
        RIGHT_TWO_OBJECT,
        THREE_OBJECT_CLOSE_BALANCE,
        ONE_OBJECT_BALANCE,
        TWO_OBJECT_BALANCE,
        ONE_OBJECT_DRIVE_AWAY,
        TWO_OBJECT_BRIDGE_BALANCE,
        TWO_HALF_OBJECT_BRIDGE_BALANCE,
        ONE_HALF_OBJECT_BALANCE,
        ;
    }
}
