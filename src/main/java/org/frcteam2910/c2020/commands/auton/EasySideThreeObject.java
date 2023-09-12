package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class EasySideThreeObject extends AutonCommandBase {
    public EasySideThreeObject(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public EasySideThreeObject(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        boolean isBlue = getSide(container);
        this.addCommands(
            new EasySideTwoObject(container, trajectories),
            new ParallelDeadlineGroup(
                new WaitCommand(0.25),
                new SetArmSafely(ScoreMode.CUBE_INTAKE)
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getEasySideConeToPickup2(isBlue)),
                new WaitForEndOfTrajectory(trajectories.getEasySideConeToPickup2(isBlue), 4.0,
                    new CubeIntake(intake, true, true)
                )
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getEasySideConeToPlace2(isBlue)),
                new SequentialCommandGroup(
                    new SetIntakeDeployPosition(intake, 0.0),
                    new WaitCommand(0.3),
                    new SetIntakeDeployPosition(intake, 45.0)
                ),
                new SetArmRotator(arm, 7.5),
                new SequentialCommandGroup(
                    new WaitCommand(0.3),
                    new InstantCommand(()->intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_SPIT_RPM, true))
                ),
                new WaitForEndOfTrajectory(trajectories.getEasySideConeToPlace2(isBlue), 0.5, 
                    new CubeSpit(intake, false)
                )
            ),
            new FollowTrajectoryCommand(drive, trajectories.getEasySideToEndSpot(isBlue))
        );
    }
}