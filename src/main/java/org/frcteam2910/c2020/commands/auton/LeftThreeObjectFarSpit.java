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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class LeftThreeObjectFarSpit extends AutonCommandBase {
    public LeftThreeObjectFarSpit(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getIntake());
    }

    public LeftThreeObjectFarSpit(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Intake intake) {
        boolean isBlue = getSide(container);
        resetRobotPose(container, trajectories.getThreeObjectFarPart1(isBlue));
        this.addCommands(
            new SetArmSafelyAuton(ScoreMode.CONE_MID, false, false),
            new ScoreConeAuton(intake),
            new SetArmIntakeRPM(intake, 0),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart1(isBlue)),
                new CubeIntake(intake, true)
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart2(isBlue)),
                new InstantCommand(()->intake.setCubeRollerRPM(0)),
                new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES),
                new WaitForEndOfTrajectory(
                    trajectories.getThreeObjectFarPart2(isBlue),
                    1.0,
                    new CubeSpit(intake)
                )
            ),
            new WaitCommand(0.3),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart3(isBlue)),
                new CubeIntake(intake, true)
            ),
            new WaitCommand(0.2),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart4B(isBlue)),
                new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES),
                new SequentialCommandGroup(
                    new SetArmSafely(ScoreMode.HOME),
                    new WaitForEndOfTrajectory(
                        trajectories.getThreeObjectFarPart4B(isBlue),
                        2,
                        new SetArmSafely(ScoreMode.CUBE_MID)
                    )
                )
            ),
            new ScoreCubeAuton(intake),
            new SetUpForTeleop(intake)
        );
    }
}