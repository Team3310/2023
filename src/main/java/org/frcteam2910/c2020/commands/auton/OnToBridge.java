package org.frcteam2910.c2020.commands.auton;

import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.DriveBalanceCommand;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

public class OnToBridge extends AutonCommandBase {
    public OnToBridge(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public OnToBridge(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.getOnToBridge());
        this.addCommands(
            new FollowTrajectoryCommand(drive, trajectories.getOnToBridge()),
            new DriveBalanceCommand(drive, true, false)
        );
    }
}