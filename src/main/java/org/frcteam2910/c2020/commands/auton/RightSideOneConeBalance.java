package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

public class RightSideOneConeBalance extends AutonCommandBase {
    public RightSideOneConeBalance(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public RightSideOneConeBalance(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        this.addCommands(
            new RightSideTwoCone(container, trajectories),
            new FollowTrajectoryCommand(drive, trajectories.getEasySideToBridge1(getSide(container))),
            new OnToBridge(container, trajectories,-5)
        );
    }
}