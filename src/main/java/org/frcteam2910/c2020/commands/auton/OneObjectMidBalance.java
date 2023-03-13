package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.BooleanSupplier;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class OneObjectMidBalance extends AutonCommandBase {
    public OneObjectMidBalance(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public OneObjectMidBalance(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        this.addCommands(
            new OneObjectMid(container, trajectories),
            new FollowTrajectoryCommand(drive, trajectories.getOnToBridge()),
            new FollowTrajectoryCommand(drive, trajectories.getUpBridge(drive.getPose().translation, drive.getPose().rotation, 8)),
            new WaitCommand(1.0),
            new FollowTrajectoryCommand(drive, trajectories.getUpBridge(drive.getPose().translation, drive.getPose().rotation, -12)),
            new DriveBalanceCommand(drive, true, false)
            // new FollowTrajectoryCommand(drive, trajectories.getOnToBridge()),
            // new WaitCommand(2.0),
            // new FollowTrajectoryCommand(drive, trajectories.getPastBridge())
            // // new FollowTrajectoryCommand(drive, trajectories.getUpBridge(drive.getPose().translation, drive.getPose().rotation, -45)),
            // // new DriveBalanceCommand(drive, true, false)
        );
    }
}