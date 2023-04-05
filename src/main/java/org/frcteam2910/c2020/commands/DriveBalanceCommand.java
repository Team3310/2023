package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.ArrayList;

import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;

public class DriveBalanceCommand extends CommandBase {
    private final DrivetrainSubsystem drive;

    private final boolean isSlow;
    private Vector2 start;
    private ArrayList<Double> lastAngles = new ArrayList<Double>();

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
        //drive.setBalanceInitialPos(start);
        start = drive.getPose().translation;
        drive.getBalanceTimer().start();
    }

    @Override
    public boolean isFinished(){
        // If the distance we traveled from initial position > distance, go slower
        double minTraveledInches = 10.0;
        if(lastAngles.size()<5){
            lastAngles.add(0, drive.getRollDegreesOffLevel());
        }
        else{
            lastAngles.remove(4);
            lastAngles.add(0, drive.getRollDegreesOffLevel());
        }
        double distanceTravelled = start.subtract(drive.getPose().translation).length;
        //boolean metTarget = drivetrain.getPitch()>180 ? (360-drivetrain.getPitch())<this.deadband:drivetrain.getPitch()<this.deadband;
        boolean isBalanced = drive.isBalanced();
        double averageAngle=0.0;
        for(int i=0; i<lastAngles.size();i++){
            averageAngle+=lastAngles.get(i);
        }
        averageAngle/=5;
        boolean isFalling = drive.getStartDegrees()-1>averageAngle;
        //TODO readd the condition for distance travelled
        return (isSlow?isBalanced:distanceTravelled>minTraveledInches&&isFalling);
    }

    @Override
    public void end(boolean interrupted) {
        if(isSlow)
            drive.setDriveControlMode(DriveControlMode.HOLD);      
        drive.setBalanceInitialPos(Vector2.ZERO);
        drive.setDriveBrake();
        drive.resetPose(new RigidTransform2(new Vector2(-164, drive.getPose().translation.y), drive.getPose().rotation));
        drive.drive(Vector2.ZERO, 0.0, false);
        drive.getBalanceTimer().stop();
        drive.getBalanceTimer().reset();
    }
}
