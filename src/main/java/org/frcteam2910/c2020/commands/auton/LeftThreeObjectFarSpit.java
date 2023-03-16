package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.SideChooser.SideMode;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LeftThreeObjectFarSpit extends AutonCommandBase {
    public LeftThreeObjectFarSpit(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public LeftThreeObjectFarSpit(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive) {
        boolean isBlue = getSide(container);
        resetRobotPose(container, trajectories.getThreeObjectFarPart2(isBlue));
        this.addCommands(
            //new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart1()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart2(isBlue)),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart3(isBlue)),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart4(isBlue)),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart5(isBlue))
        );
    }
}