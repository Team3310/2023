package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RightSidecone extends AutonCommandBase {
    public RightSidecone(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public RightSidecone(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.getConeBridgefromPlaceToLoad());
        this.addCommands(
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPlaceToLoad()),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgeLoadTOPickup1()),
            new WaitCommand(2.0),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPickUp1ToLoad()),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromLoadtoPlace()),
            new WaitCommand(2.0),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPlaceToLoad()),
            new debugAutonPosition(drive,"1"),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgeLoadToPickup2()),
            new debugAutonPosition(drive, "2"),
            new WaitCommand(2.0),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPickUp2ToLoad()),
            new debugAutonPosition(drive,"3"),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromLoadtoPlace()),
            new debugAutonPosition(drive,"4")
        );
    }
}