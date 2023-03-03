package org.frcteam2910.c2020.commands.auton;

import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RightThreeObject extends AutonCommandBase {
    public RightThreeObject(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public RightThreeObject(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.getThreeObjectBridgePart1());
        this.addCommands(
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectBridgePart1()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectBridgePart2()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectBridgePart3()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectBridgePart4())
        );
    }
}