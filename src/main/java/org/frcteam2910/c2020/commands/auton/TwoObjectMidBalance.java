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
            new WaitCommand(0.3),
            new ParallelCommandGroup(
                new InstantCommand(()->drive.setBridgeDriveVoltage(2)),
                new setArmSafeAuton(ScoreMode.CONE_INTAKE),
                new SetIntakeRPM(intake, Constants.INTAKE_COLLECT_RPM)
            ),
            new ParallelRaceGroup(
                new WaitUntilCommand(()->intake.getConeSensor().get()),
                new WaitCommand(2.0)
            ),    
            new ParallelCommandGroup(
                new InstantCommand(()->drive.setBridgeDriveVoltage(-4)),
                new setArmSafeAuton(true),
                new SetIntakeRPM(intake, 0)
            ),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new ChangeDriveMode(drive, DriveControlMode.BALANCE)
            // new FollowTrajectoryCommand(drive, trajectories.getOnToBridge()),
            // new WaitCommand(2.0),
            // new FollowTrajectoryCommand(drive, trajectories.getPastBridge())
            // // new FollowTrajectoryCommand(drive, trajectories.getUpBridge(drive.getPose().translation, drive.getPose().rotation, -45)),
            // // new DriveBalanceCommand(drive, true, false)
        );
    }
}