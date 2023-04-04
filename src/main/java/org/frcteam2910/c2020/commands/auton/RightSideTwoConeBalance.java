package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class RightSideTwoConeBalance extends AutonCommandBase {
    public RightSideTwoConeBalance(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public RightSideTwoConeBalance(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        this.addCommands(  
        new RightSideTwoCone(container, trajectories),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getToBridge1(getSide(container))),
                new SetArmSafely(ScoreMode.HOME),
                new SetIntakeRPM(intake, 0)
            ),
            new OnToBridge(container, trajectories, -5),
            new InstantCommand(()->SmartDashboard.putNumber("finished auton Time", DriverStation.getMatchTime()))
        );
    }
}