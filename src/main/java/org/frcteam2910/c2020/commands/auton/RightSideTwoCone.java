package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class RightSideTwoCone extends AutonCommandBase {
    public RightSideTwoCone(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public RightSideTwoCone(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        boolean isBlue = getSide(container);
        SmartDashboard.putBoolean("isBlue", isBlue);
        resetRobotPose(container, trajectories.getConeBridgeToPickup1(isBlue));
        this.addCommands(
            new SetArmExtender(arm, 0.0),
            new SetArmRotator(arm, ScoreMode.MID.getAngle()-7.0),
            new SetArmExtender(arm, ScoreMode.MID.getInches()),
            new SetIntakeRPM(intake, Constants.ARM_INTAKE_SPIT_RPM),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(()->!intake.getConeSensor().get()),
                    new WaitCommand(0.05)
                ),
                new WaitCommand(0.5)
            ),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(drive, trajectories.getConeBridgeToPickup1(isBlue)),
                new InstantCommand(()->intake.setCubeIntakeDeployTargetPosition(110)),
                new SequentialCommandGroup(
                    new WaitCommand(0.75),
                    new SetIntakeRPM(intake, Constants.ARM_CUBE_INTAKE_COLLECT_RPM),
                    new SetArmSafely(ScoreMode.CUBE_INTAKE, false, false)
                )
            ),
            new WaitCommand(0.1),
            new ParallelCommandGroup(
                new InstantCommand(()->intake.setCubeRollerRPM(0)),
                new InstantCommand(()->intake.setArmIntakeHold()),
                new FollowTrajectoryCommand(drive, trajectories.getConeBridgeToPlace1(isBlue)),
                new InstantCommand(()->intake.setCubeIntakeDeployTargetPosition(0)),
                new SequentialCommandGroup(
                    new SetArmExtender(arm, 0.0),
                    new SetArmRotator(arm, ScoreMode.MID.getAngle()+1),
                    new SetArmExtender(arm, 10)
                )
            ),
            new SetIntakeRPM(intake, Constants.ARM_CUBE_INTAKE_SPIT_RPM),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(()->!intake.getCubeSensor().get()),
                    new WaitCommand(0.2)
                ),
                new WaitCommand(0.75)
            )
        //     new SetArmSafelyAuton(ScoreMode.HIGH),
        //     new SetIntakeRPM(intake, Constants.INTAKE_SPIT_RPM),
        //     new ParallelRaceGroup(
        //         new SequentialCommandGroup(
        //             new WaitUntilCommand(()->!intake.getConeSensor().get()),
        //             new WaitCommand(0.3)
        //         ),
        //         new WaitCommand(1.0)
        //     ),
        //     new ParallelDeadlineGroup(
        //         new FollowTrajectoryCommand(drive, trajectories.getConeBridgeToPickup1(isBlue)),  
        //         new SetIntakeRPM(intake, 0),
        //         new SequentialCommandGroup(
        //             new SetArmSafely(ScoreMode.CUBE_INTAKE), //TODO once they add the cube intake to back change
        //             new SetIntakeRPM(intake, -Constants.INTAKE_COLLECT_RPM)
        //         )
        //     ),
        //     new ParallelRaceGroup(
        //         new SequentialCommandGroup(
        //             new WaitUntilCommand(()->intake.getCubeSensor().get()),
        //             new WaitCommand(0.3)
        //         ),
        //         new WaitCommand(1.0)
        //     ),
        //     new ParallelDeadlineGroup(
        //         new FollowTrajectoryCommand(drive, trajectories.getConeBridgeToPlace1(isBlue)),  
        //         new SetArmSafely(ScoreMode.ZERO)
        //     ),
        //     new SetArmSafely(ScoreMode.HIGH),
        //     new SetIntakeRPM(intake, -Constants.INTAKE_SPIT_RPM),
        //     new ParallelRaceGroup(
        //         new SequentialCommandGroup(
        //             new WaitUntilCommand(()->!intake.getCubeSensor().get()),
        //             new WaitCommand(0.3)
        //         ),
        //         new WaitCommand(1.0)
        //     ),
        //     new SetArmSafely(ScoreMode.ZERO)
        );
    }
}