package org.frcteam2910.c2020.commands.auton;

import java.time.Instant;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class MidTwoBalance extends AutonCommandBase {
    public MidTwoBalance(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public MidTwoBalance(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        this.addCommands(
            new OneObjectMid(container, trajectories),
            new InstantCommand(()->{
                drive.setBridgeDriveVoltage(5);
                drive.setCommandedGyroAngle(180);
            }),
            new ChangeDriveMode(drive, DriveControlMode.BRIDGE_VOLTAGE),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()<5),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()<5),
                new CubeIntake(intake, true),
                new InstantCommand(()->drive.setBridgeDriveVoltage(3))
            ),
            new ParallelRaceGroup(
                new WaitCommand(1.8),
                new WaitUntilCommand(()->intake.getCubeSensor().get())
            ),
            new InstantCommand(()->{
                drive.setCommandedGyroAngle(0);
                intake.setCubeIntakeDeployTargetPosition(Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES);
                intake.setCubeRollerRPM(0, true);
                drive.setBridgeDriveVoltage(-4);
            }),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new InstantCommand(() -> drive.setBalanceStartDegrees(drive.getRollDegreesOffLevel())),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new DriveBalanceCommand(drive, false),
                    new DriveBalanceCommand(drive, true)
                ),
                new SequentialCommandGroup(
                    new InstantCommand(()->intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_SPIT_RPM, true)),
                    new WaitCommand(0.5),
                    new CubeSpit(intake, true)
                )
            )
            // new FollowTrajectoryCommand(drive, trajectories.getOnToBridge()),
            // new WaitCommand(2.0),
            // new FollowTrajectoryCommand(drive, trajectories.getPastBridge())
            // // new FollowTrajectoryCommand(drive, trajectories.getUpBridge(drive.getPose().translation, drive.getPose().rotation, -45)),
            // // new DriveBalanceCommand(drive, true, false)
        );
    }
}