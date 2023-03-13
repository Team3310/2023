package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.common.control.Trajectory;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LeftTwoObjectBalance extends AutonCommandBase {
    public LeftTwoObjectBalance(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public LeftTwoObjectBalance(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.getThreeObjectClosePart1());
        this.addCommands(
            new LeftTwoObjectClose(container, trajectories),
            new FollowTrajectoryCommand(drive, trajectories.getUpBridge(drive.getPose().translation, drive.getPose().rotation)),
            new DriveBalanceCommand(drive, true, false)
        );
    }
}