package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.Rotation2;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroGyroscope extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    public ZeroGyroscope(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        setName("Zero Gyroscope");
    }

    @Override
    public void initialize() {
        drivetrain.resetGyroAngle(Rotation2.ZERO);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
