package org.frcteam2910.c2020.commands.auton;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class OneObjectMidMobilityBalance extends AutonCommandBase {
    public OneObjectMidMobilityBalance(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public OneObjectMidMobilityBalance(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        this.addCommands(
            new OneObjectMid(container, trajectories),
            new InstantCommand(()->drive.setBridgeDriveVoltage(-5)),
            new ChangeDriveMode(drive, DriveControlMode.BRIDGE_VOLTAGE),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()<5),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()<5),
            new InstantCommand(()->drive.setBridgeDriveVoltage(1)),
            new ParallelDeadlineGroup(
                new WaitCommand(0.9),
                new CubeIntake(intake, true)
            ),
            new WaitCommand(0.2),
            new InstantCommand(()->{
                intake.setCubeIntakeDeployTargetPosition(Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES);
                drive.setBridgeDriveVoltage(6);
            }),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new InstantCommand(() -> drive.setBalanceStartDegrees(drive.getRollDegreesOffLevel())),
            new DriveBalanceCommand(drive, true)
            // new FollowTrajectoryCommand(drive, trajectories.getOnToBridge()),
            // new WaitCommand(2.0),
            // new FollowTrajectoryCommand(drive, trajectories.getPastBridge())
            // // new FollowTrajectoryCommand(drive, trajectories.getUpBridge(drive.getPose().translation, drive.getPose().rotation, -45)),
            // // new DriveBalanceCommand(drive, true, false)
        );
    }
}