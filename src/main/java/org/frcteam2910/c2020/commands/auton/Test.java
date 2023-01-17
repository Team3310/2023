package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Test extends AutonCommandBase {
    public Test(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public Test(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.getTestPart1());
        this.addCommands(
            new FollowTrajectoryCommand(drive, trajectories.getTestPart1()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getTestPart2()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getTestPart3()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getTestPart4()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getTestPart5())
        );
    }
}