package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ZeroAllWheels extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    public ZeroAllWheels(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setModulesAngle(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}