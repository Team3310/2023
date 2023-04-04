package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class OnToBridge extends AutonCommandBase {
    public 
    OnToBridge(RobotContainer container, AutonomousTrajectories trajectories, double value){
        this(container, trajectories, value, container.getDrivetrainSubsystem());
    }

    public OnToBridge(RobotContainer container, AutonomousTrajectories trajectories, double value, DrivetrainSubsystem drive) {
        this.addCommands(
            new ChangeDriveMode(drive, DriveControlMode.BRIDGE_VOLTAGE),
            new InstantCommand(()->drive.setBridgeDriveVoltage(value)),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()>20),
            new WaitUntilCommand(() -> drive.getRollDegreesOffLevel()<12.25),
            new DriveBalanceCommand(drive, true, false, false),
            new WaitCommand(0.1),
            new DriveBalanceCommand(drive, true, false, true)
        );
    }
}