package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PlaceConeAndLeave extends AutonCommandBase {
    public PlaceConeAndLeave(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public PlaceConeAndLeave(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.getOnToBridge());
        this.addCommands(
            new OneObjectMid(container, trajectories),
            new FollowTrajectoryCommand(drive, trajectories.placeAndLeave())
        );
    }
}