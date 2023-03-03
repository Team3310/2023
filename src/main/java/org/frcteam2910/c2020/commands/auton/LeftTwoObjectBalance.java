package org.frcteam2910.c2020.commands.auton;

import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.DriveBalanceCommand;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

public class LeftTwoObjectBalance extends AutonCommandBase {
    public LeftTwoObjectBalance(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public LeftTwoObjectBalance(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.getThreeObjectClosePart1());
        this.addCommands(
            new LeftTwoObjectClose(container, trajectories),
            new FollowTrajectoryCommand(drive, trajectories.getUpBirdge(drive.getPose().translation, drive.getPose().rotation)),
            new DriveBalanceCommand(drive, true, false)
        );
    }
}