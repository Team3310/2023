package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.Rotation2;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ZeroAll extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    public ZeroAll(DrivetrainSubsystem drivetrain) {
        // this.balanceElevator = balanceElevator;
        // this.climbElevator = climbElevator;
        this.drivetrain = drivetrain;
        
        // addRequirements(this.balanceElevator);
        // addRequirements(this.climbElevator);
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        // balanceElevator.setElevatorZero();
        // climbElevator.setElevatorZero(Constants.ELEVATOR_HOME_POSITION);
        drivetrain.resetGyroAngle(Rotation2.ZERO);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}