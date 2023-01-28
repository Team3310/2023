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
            //new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart1()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPlaceToLoad()),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgePickup1()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPickUp1ToLoad()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromLoadtoPlace()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPlaceToLoad()),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgePickup2()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPickUp2ToLoad()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromLoadtoPlace())
        );
    }
}