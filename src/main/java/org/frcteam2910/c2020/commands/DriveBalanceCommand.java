package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.common.math.Vector2;

public class DriveBalanceCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    
    private final boolean isAuton;

    public DriveBalanceCommand(DrivetrainSubsystem drivetrain, boolean isAuton) {
        this.drivetrain = drivetrain;
        this.isAuton=isAuton;

        drivetrain.setBalanceInitialPos(drivetrain.getPose().translation);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setDriveControlMode(DriveControlMode.BALANCE);
    }

    @Override
    public boolean isFinished(){
        // If the distance we traveled from initial position > distance, go slower
        int minTraveledInches = 10;
        int maxTraveledInches = 40;
        boolean traveledFarEnough = Math.abs(drivetrain.getPose().translation.subtract(drivetrain.getBalanceInitialPos()).x) > minTraveledInches;
        boolean traveledTooFar = Math.abs(drivetrain.getPose().translation.subtract(drivetrain.getBalanceInitialPos()).x) > maxTraveledInches;
        //boolean metTarget = drivetrain.getPitch()>180 ? (360-drivetrain.getPitch())<this.deadband:drivetrain.getPitch()<this.deadband;
        boolean isBalanced = drivetrain.isBalanced();
        return /*metTarget &&*/ (traveledFarEnough && !traveledTooFar && isBalanced) || drivetrain.getDriveControlMode()==DriveControlMode.JOYSTICKS;
    }

    @Override
    public void end(boolean interrupted) {
        if(isAuton)
            drivetrain.setDriveControlMode(DriveControlMode.TRAJECTORY);
        else    
            drivetrain.setDriveControlMode(DriveControlMode.JOYSTICKS);     
        SmartDashboard.putNumber("Traveled BInches", Math.abs(drivetrain.getPose().translation.subtract(drivetrain.getBalanceInitialPos()).x));
        drivetrain.setBalanceInitialPos(Vector2.ZERO);
        drivetrain.drive(Vector2.ZERO, 0.0, false);
    }
}
