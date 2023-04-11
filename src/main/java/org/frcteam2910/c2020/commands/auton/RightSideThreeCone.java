package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RightSideThreeCone extends AutonCommandBase {
    public RightSideThreeCone(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public RightSideThreeCone(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        boolean isBlue = false;//getSide(container);
        this.addCommands(
            new RightSideTwoCone(container, trajectories),
            new ParallelDeadlineGroup(
                new WaitCommand(0.1),
                new SetArmSafely(ScoreMode.CUBE_INTAKE)
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getEasySideConeToPickup2(isBlue)),
                new WaitForEndOfTrajectory(trajectories.getEasySideConeToPickup2(isBlue), 2.5,
                    new CubeIntake(intake, true)
                )
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getEasySideConeToPlace2(isBlue)),
                new WaitForEndOfTrajectory(trajectories.getEasySideConeToPlace2(isBlue), 1.5, 
                    new SetArmSafely(ScoreMode.CUBE_MID)
                )
            ),
            new ScoreCubeAuton(intake)
        );
    }
}