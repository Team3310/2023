package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class TwoObjectMidBalance extends AutonCommandBase {
    public TwoObjectMidBalance(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public TwoObjectMidBalance(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        this.addCommands(
            new OneObjectMid(container, trajectories),
            new ChangeDriveMode(drive, DriveControlMode.BRIDGE_VOLTAGE),
            new InstantCommand(()->drive.setBridgeDriveVoltage(5)),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()<5),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()<5),
            new ChangeDriveMode(drive, DriveControlMode.TRAJECTORY),
            new TwoObjectMidBalancePart2(container, trajectories)
            // new FollowTrajectoryCommand(drive, trajectories.getOnToBridge()),
            // new WaitCommand(2.0),
            // new FollowTrajectoryCommand(drive, trajectories.getPastBridge())
            // // new FollowTrajectoryCommand(drive, trajectories.getUpBridge(drive.getPose().translation, drive.getPose().rotation, -45)),
            // // new DriveBalanceCommand(drive, true, false)
        );
    }

    private class TwoObjectMidBalancePart2 extends AutonCommandBase{
        public TwoObjectMidBalancePart2(RobotContainer container, AutonomousTrajectories trajectories){
            this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
        }

        public TwoObjectMidBalancePart2(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
            resetRobotPose(container, trajectories.getFromOverBridgeToCone());
            this.addCommands(
                new ParallelDeadlineGroup(
                    new FollowTrajectoryCommand(drive, trajectories.getFromOverBridgeToCone()),
                    new SetIntakeRPM(intake, Constants.ARM_INTAKE_COLLECT_RPM),
                    new SetServosOut(intake),
                    new SetArmSafelyAuton(ScoreMode.CONE_INTAKE)
                ),
                new ParallelRaceGroup(
                    new WaitUntilCommand(()->intake.getConeSensor().get()),
                    new WaitCommand(1.0)
                ),
                new ParallelDeadlineGroup(
                    new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
                    new InstantCommand(()->drive.setBridgeDriveVoltage(-5)), // set voltage first in case of any previous value
                    new ChangeDriveMode(drive, DriveControlMode.BRIDGE_VOLTAGE),
                    new SetIntakeRPM(intake, 0),
                    new SequentialCommandGroup(
                        new SetArmSafelyAuton(true),
                        new SetServosIn(intake)
                    )
                ),
                new ChangeDriveMode(drive, DriveControlMode.BALANCE)
            );
        }
    }
}

