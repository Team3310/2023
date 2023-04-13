package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class EasyThreeObject extends AutonCommandBase {
    public EasyThreeObject(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public EasyThreeObject(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive) {
        boolean isBlue = getSide(container);
        resetRobotPose(container, trajectories.getThreeObjectBridgePart1(isBlue));
        this.addCommands(
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectBridgePart1(isBlue)),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectBridgePart2(isBlue)),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectBridgePart3(isBlue)),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectBridgePart4(isBlue))
        );
    }
}