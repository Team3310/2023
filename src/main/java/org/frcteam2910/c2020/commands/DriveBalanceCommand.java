package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.ArrayList;

import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.common.math.Vector2;

public class DriveBalanceCommand extends CommandBase {
    private final DrivetrainSubsystem drive;

    private final boolean isSlow;
    private double lastValue;
    private ArrayList<Double> lastValues = new ArrayList<>();

    public DriveBalanceCommand(DrivetrainSubsystem drivetrain, boolean isSlow) {
        this.drive = drivetrain;
        this.isSlow = isSlow;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        lastValue=drive.getZGravityVector();
        drive.setBalanceStartDegrees(drive.getRollDegreesOffLevel());
        drive.setSlowBalance(isSlow);
        drive.setDriveControlMode(DriveControlMode.BALANCE);
        drive.getBalanceTimer().start();
    }

    @Override
    public boolean isFinished(){
        boolean isBalanced = drive.isBalanced();
        boolean isFalling = drive.getZGravityVector()-lastValue>0.00198;
        lastValue = drive.getZGravityVector();
        return (isSlow?isBalanced:isFalling);

        // if(lastValues.size()<5){
        //     lastValues.add(drive.getZGravityVector());
        // }else{
        //     lastValues.remove(0);
        //     lastValues.add(drive.getZGravityVector());
        // }

        // double averageValue=0.0;
        // for(int i=0; i<lastValues.size();i++){
        //     averageValue+=lastValues.get(i);
        // }
        // averageValue/=lastValues.size();
        // boolean isFalling = averageValue > 0.001;
        // return (isSlow?isBalanced:isFalling);
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
