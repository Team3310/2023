package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class RightSideTwoConeBalance extends AutonCommandBase {
    public RightSideTwoConeBalance(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public RightSideTwoConeBalance(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        this.addCommands(  
        new RightSideTwoCone(container, trajectories),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getEasySideToBridge1(getSide(container))),
                new SetArmSafelyAuton(ScoreMode.HOME, false, false),
                new SetArmIntakeRPM(intake, 0, true)
            ),
            new OnToBridge(container, trajectories, -5)
        );
    }
}