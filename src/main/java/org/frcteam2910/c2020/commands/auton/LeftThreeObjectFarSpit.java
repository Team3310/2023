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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class LeftThreeObjectFarSpit extends AutonCommandBase {
    public LeftThreeObjectFarSpit(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getIntake());
    }

    public LeftThreeObjectFarSpit(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Intake intake) {
        boolean isBlue = getSide(container);
        resetRobotPose(container, trajectories.getThreeObjectFarPart1(false));
        this.addCommands(
            new SetArmSafely(ScoreMode.CONE_MID),
            new SetArmIntakeRPM(intake, Constants.ARM_CONE_INTAKE_SPIT_RPM),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(()->!intake.getConeSensor().get()),
                    new WaitCommand(0.3)
                ),
                new WaitCommand(0.75)
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart1(false)),
                new SequentialCommandGroup(
                    new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES),
                    new SetArmIntakeRPM(intake, 0)
                ),
                new SetArmSafely(ScoreMode.HOME),
                new InstantCommand(()->{
                    intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_COLLECT_RPM);
                    intake.stopRollingOnTriggeredCubeIntakeDIO = true;
                }) 
            ),
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart2(false)),
                new InstantCommand(()->intake.setCubeRollerRPM(0)),
                new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES)
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                new InstantCommand(()->intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_SPIT_RPM))
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart3(false)),
                new SetArmSafely(ScoreMode.CUBE_INTAKE),
                new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES),
                new SetArmIntakeRPM(intake, Constants.ARM_CUBE_INTAKE_COLLECT_RPM),
                new InstantCommand(()->{
                    intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_COLLECT_RPM);
                    intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                    intake.stopRollingOnTriggeredArmIntakeDIO = true;
                })
            ),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart4B(false)),
                new SequentialCommandGroup(
                    new SetArmSafely(ScoreMode.HOME),
                    new WaitCommand(0.6),
                    new SetArmSafely(ScoreMode.CUBE_MID)
                ),  
                new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES),
                new SetArmIntakeRPM(intake, 0),
                new InstantCommand(()->{
                    intake.setCubeRollerRPM(0);
                    intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                    intake.stopRollingOnTriggeredArmIntakeDIO = false;
                })
            ),
            new SetArmIntakeRPM(intake, Constants.ARM_CUBE_INTAKE_SPIT_RPM),
            new WaitCommand(0.3),
            new SetArmSafely(ScoreMode.HOME),
            new SetArmIntakeRPM(intake, 0)
        );
    }
}