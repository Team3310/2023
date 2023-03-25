package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

public class OnToBridge extends AutonCommandBase {
    public OnToBridge(RobotContainer container, AutonomousTrajectories trajectories, double value){
        this(container, trajectories, value, container.getDrivetrainSubsystem());
    }

    public OnToBridge(RobotContainer container, AutonomousTrajectories trajectories, double value, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.getOnToBridge());
        this.addCommands(
            new ChangeDriveMode(drive, DriveControlMode.BRIDGE_VOLTAGE),
            new InstantCommand(()->drive.setBridgeDriveVoltage(value)),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>15),
            new DriveBalanceCommand(drive, true, false)
        );
    }
}