package org.frcteam2910.c2020.commands;

import org.frcteam2910.common.math.Rotation2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.RigidTransform2;

public class ResetOdometryHeading extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;
    private boolean finished;

    public ResetOdometryHeading(DrivetrainSubsystem drivetrain) {
        drivetrainSubsystem = drivetrain;
        finished = false;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO);
        drivetrainSubsystem.resetPose(RigidTransform2.ZERO);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        finished = true;
    }

}
