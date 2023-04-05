package org.frcteam2910.c2020.commands.auton;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RightSideTwoCone extends AutonCommandBase {
    public RightSideTwoCone(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public RightSideTwoCone(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        boolean isBlue = getSide(container);
        resetRobotPose(container, trajectories.getEasySideConeToPickup1(isBlue));
        this.addCommands(
            new SetArmSafelyAuton(ScoreMode.CONE_MID, false, false),
            new ScoreConeAuton(intake),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(drive, trajectories.getEasySideConeToPickup1(isBlue)),
                new CubeIntakeAuton(intake, true, trajectories.getEasySideConeToPickup1(isBlue))
            ),
            new WaitCommand(0.1),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(drive, trajectories.getEasySideConeToPlace1(isBlue)),
                new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES),
                new InstantCommand(()->{
                    intake.setCubeRollerRPM(0);
                    intake.setArmIntakeHoldPosition();
                }),
                new SetArmSafelyAuton(ScoreMode.CONE_MID, false, false)
            ),
            new ScoreCubeAuton(intake)
        );
    }
}