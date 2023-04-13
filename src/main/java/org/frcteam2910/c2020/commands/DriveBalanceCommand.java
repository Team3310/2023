package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.common.math.Vector2;

public class DriveBalanceCommand extends CommandBase {
    private final DrivetrainSubsystem drive;

    private final boolean isSlow;
    private double minGravityVector = 0.96450;

    public DriveBalanceCommand(DrivetrainSubsystem drivetrain, boolean isSlow) {
        this.drive = drivetrain;
        this.isSlow = isSlow;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drive.setBalanceStartDegrees(drive.getRollDegreesOffLevel());
        drive.setSlowBalance(isSlow);
        drive.setDriveControlMode(DriveControlMode.BALANCE);
        drive.getBalanceTimer().start();
    }

    @Override
    public boolean isFinished(){
        boolean isBalanced = drive.isBalanced();
        boolean isFalling = drive.getZGravityVector()>minGravityVector;//drive.getZGravityVector()-minGravityVector>0.001;
        return (isSlow?isBalanced:isFalling);

        // double averageAngle=0.0;
        // for(int i=0; i<lastAngles.size();i++){
        //     averageAngle+=lastAngles.get(i);
        // }
        // averageAngle/=5;
        // boolean isFalling = drive.getStartDegrees()-1>averageAngle;
        // return (isSlow?isBalanced:distanceTravelled>minTraveledInches&&isFalling);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(Vector2.ZERO, 0.0, false);
        drive.setDriveControlMode(DriveControlMode.HOLD);      
        drive.setBalanceInitialPos(Vector2.ZERO);
        drive.setDriveBrake();
        drive.getBalanceTimer().stop();
        drive.getBalanceTimer().reset();
    }
}
