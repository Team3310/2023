package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;

public class AutonPositionDebug extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private String part;

    public AutonPositionDebug(DrivetrainSubsystem drivetrain, String part) {
        this.drivetrain = drivetrain;
        this.part = part;
        // addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Path X "+part,drivetrain.getFollower().getLastState().getPathState().getPosition().x);
        SmartDashboard.putNumber("Path Y "+part,drivetrain.getFollower().getLastState().getPathState().getPosition().y);
        SmartDashboard.putNumber("Actual X "+part, drivetrain.getPose().translation.x);
        SmartDashboard.putNumber("Actual Y "+part, drivetrain.getPose().translation.y);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}