package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class OneHalfObjectMidBalance extends AutonCommandBase {
    public OneHalfObjectMidBalance(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public OneHalfObjectMidBalance(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        boolean isBlue=getSide(container);
        this.addCommands(
            new OneObjectMid(container, trajectories),
            new InstantCommand(()->drive.setBridgeDriveVoltage(6)),
            new ChangeDriveMode(drive, DriveControlMode.BRIDGE_VOLTAGE),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()<5),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()<5),
            new InstantCommand(()->drive.setBridgeDriveVoltage(4)),
            new InstantCommand(()->drive.setBridgeDriveAngle(-10)),
            new WaitCommand(1.0),
            new InstantCommand(()->drive.setBridgeDriveAngle(0)),
            new InstantCommand(()->drive.setBridgeDriveVoltage(-6)),
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