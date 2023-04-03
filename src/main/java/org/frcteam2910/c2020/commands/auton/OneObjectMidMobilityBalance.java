package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.BooleanSupplier;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class OneObjectMidMobilityBalance extends AutonCommandBase {
    public OneObjectMidMobilityBalance(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public OneObjectMidMobilityBalance(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        boolean isBlue=getSide(container);
        this.addCommands(
            new OneObjectMid(container, trajectories),
            new InstantCommand(()->drive.setBridgeDriveVoltage(-5)),
            new ChangeDriveMode(drive, DriveControlMode.BRIDGE_VOLTAGE),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()<5),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new InstantCommand(()->{
                intake.stopRollingOnTriggeredCubeIntakeDIO = true;
                intake.setCubeRollerRPM(2000);
            }),
            new ParallelCommandGroup(
                new SetIntakeDeployPosition(intake, 110),
                new InstantCommand(()->drive.setBridgeDriveVoltage(-3.5))
            ),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()<5),
            new WaitCommand(1.1),
            new InstantCommand(()->drive.setBridgeDriveVoltage(0)),
            new WaitCommand(0.2),
            new InstantCommand(()->{
                //drive.setBridgeDriveAngle(-3);
                drive.setBridgeDriveVoltage(6);
            }),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new InstantCommand(() -> drive.setBalanceStartDegrees(drive.getRollDegreesOffLevel())),
            new DriveBalanceCommand(drive, true, true, true)
            // new FollowTrajectoryCommand(drive, trajectories.getOnToBridge()),
            // new WaitCommand(2.0),
            // new FollowTrajectoryCommand(drive, trajectories.getPastBridge())
            // // new FollowTrajectoryCommand(drive, trajectories.getUpBridge(drive.getPose().translation, drive.getPose().rotation, -45)),
            // // new DriveBalanceCommand(drive, true, false)
        );
    }
}