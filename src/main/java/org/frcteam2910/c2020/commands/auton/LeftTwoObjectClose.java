package org.frcteam2910.c2020.commands.auton;

import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LeftTwoObjectClose extends AutonCommandBase {
    public LeftTwoObjectClose(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public LeftTwoObjectClose(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.getThreeObjectClosePart1());
        this.addCommands(
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectClosePart1()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectClosePart2()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.geTthreeObjectCloseEnd1())
        );
    }
}