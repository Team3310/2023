package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
//import org.frcteam2910.c2020.subsystems.Shooter;

public class LimelightAdjustAuto extends CommandBase {
    private final DrivetrainSubsystem drive;

    public LimelightAdjustAuto(DrivetrainSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void execute(){
        drive.setDriveControlMode(DrivetrainSubsystem.DriveControlMode.LIMELIGHT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted){
        drive.setDriveControlMode(DrivetrainSubsystem.DriveControlMode.TRAJECTORY);
    }
}