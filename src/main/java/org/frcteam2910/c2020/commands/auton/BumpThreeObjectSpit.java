package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BumpThreeObjectSpit extends AutonCommandBase {
    public BumpThreeObjectSpit(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getIntake());
    }

    public BumpThreeObjectSpit(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Intake intake) {
        boolean isBlue = getSide(container);
        resetRobotPose(container, trajectories.getThreeObjectFarPart1(isBlue));
        this.addCommands(
            new SetArmSafelyAuton(ScoreMode.CONE_MID, false, false),
            new ScoreConeAuton(intake),
            new SetArmIntakeRPM(intake, 0, true),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart1(isBlue)),
                new CubeIntake(intake, true)
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart2(isBlue)),
                new SequentialCommandGroup(
                    new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES),
                    new SequentialCommandGroup(
                        new WaitCommand(0.4),
                        new InstantCommand(()->intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_SPIT_RPM, true))
                    ),
                    new WaitForEndOfTrajectory(
                        trajectories.getThreeObjectFarPart2(isBlue),
                        1.5,
                        new CubeSpit(intake, true)
                    )
                )
            ),
            new WaitCommand(0.2),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart3(isBlue)),
                new CubeIntakeAuton(intake, true, trajectories.getThreeObjectFarPart3(isBlue))//,
                // new InstantCommand(()->intake.setCubeRollerRPM(0, true))
            ),
            new WaitCommand(0.2),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart4A(isBlue)),
                new SequentialCommandGroup(
                    new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES),
                    new WaitForEndOfTrajectory(
                        trajectories.getThreeObjectFarPart4A(isBlue),
                        3.5,
                        new InstantCommand(()->intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_SPIT_RPM, true))
                    ),
                    new WaitForEndOfTrajectory(
                        trajectories.getThreeObjectFarPart4A(isBlue),
                        1.5,
                        new CubeSpit(intake, true)
                    )
                )
            ),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart5(isBlue))
        );
    }
}