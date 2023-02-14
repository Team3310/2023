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
        autonomousModeChooser.addOption("Left Three Object Close", AutonomousMode.THREE_OBJECT_CLOSE);
        autonomousModeChooser.addOption("Right Three Object", AutonomousMode.THREE_OBJECT_BRIDGE);
        autonomousModeChooser.addOption("Right 3 Cone", AutonomousMode.CONE_BRIDGE);
        autonomousModeChooser.addOption("On To Bridge", AutonomousMode.TO_BRIDGE);
        autonomousModeChooser.addOption("Up Bridge", AutonomousMode.UP_BRIDGE);
        autonomousModeChooser.addOption("Bridge Balance", AutonomousMode.BALANCE);
        autonomousModeChooser.addOption("7 Feet", AutonomousMode.SEVEN_FEET);
        autonomousModeChooser.addOption("sCurve", AutonomousMode.S_CURVE);
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
                return new LeftThreeObjectClose(container, trajectories);
            case THREE_OBJECT_BRIDGE:
                return new RightThreeObject(container, trajectories); 
            case CONE_BRIDGE:
                return new RightSidecone(container, trajectories);
            case TO_BRIDGE:
                return new OnToBridge(container, trajectories);
            case BALANCE:
                return new DriveBalanceCommand(container.getDrivetrainSubsystem(), true, false);
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
        CONE_BRIDGE,
        TO_BRIDGE,
        UP_BRIDGE,
        BALANCE
        ;
    }
}
