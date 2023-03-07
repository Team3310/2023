package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;

public class ChangeDriveMode extends CommandBase {
    private DrivetrainSubsystem drive;
    private DrivetrainSubsystem.DriveControlMode mode;

    public ChangeDriveMode(DrivetrainSubsystem drive, DrivetrainSubsystem.DriveControlMode mode) {
        this.drive = drive;
        this.mode = mode;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setDriveControlMode(mode);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
