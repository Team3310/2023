package org.frcteam2910.c2020.commands.auton;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class EasySideTwoObject extends AutonCommandBase {
    public EasySideTwoObject(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public EasySideTwoObject(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        boolean isBlue = getSide(container);
        resetRobotPose(container, trajectories.getEasySideConeToPickup1(isBlue));
        this.addCommands(
            new SetArmSafelyAuton(ScoreMode.CONE_MID, false, true),
            new ScoreConeAuton(intake),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getEasySideConeToPickup1(isBlue)),
                new CubeIntake(intake, true)
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getEasySideConeToPlace1(isBlue)),
                new InstantCommand(()->intake.setCubeRollerRPM(0, true)),
                new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES),
                new WaitForEndOfTrajectory(trajectories.getEasySideConeToPlace1(isBlue), 1.5,
                    new SetArmSafely(ScoreMode.CUBE_MID) 
                )
            ),
            // new WaitCommand(0.1),
            new ScoreCubeAuton(intake)
        );
    }
}