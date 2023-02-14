package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;

public class DriveBalanceCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    private final boolean isAuton;
    private final boolean setHold;

    public DriveBalanceCommand(DrivetrainSubsystem drivetrain, boolean isAuton, boolean setHold) {
        this.drivetrain = drivetrain;
        this.isAuton=isAuton;
        this.setHold=setHold;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setBalanceInitialPos(drivetrain.getPose().translation);
        drivetrain.setDriveControlMode(DriveControlMode.BALANCE);
    }

    @Override
    public boolean isFinished(){
        // If the distance we traveled from initial position > distance, go slower
        double minTraveledInches = 5.0;
        double maxTraveledInches = 40.0;
        boolean traveledFarEnough = Math.abs(drivetrain.getPose().translation.subtract(drivetrain.getBalanceInitialPos()).x) > minTraveledInches;
        boolean traveledTooFar = Math.abs(drivetrain.getPose().translation.subtract(drivetrain.getBalanceInitialPos()).x) > maxTraveledInches;
        //boolean metTarget = drivetrain.getPitch()>180 ? (360-drivetrain.getPitch())<this.deadband:drivetrain.getPitch()<this.deadband;
        boolean isBalanced = drivetrain.isBalanced();
        //TODO readd the condition for distance travelled
        return /*metTarget &&*/ (/*traveledFarEnough && !traveledTooFar &&*/ isBalanced) || drivetrain.getDriveControlMode()==DriveControlMode.JOYSTICKS;
    }

    @Override
    public void end(boolean interrupted) {
        if(isAuton)
            drivetrain.setDriveControlMode(DriveControlMode.TRAJECTORY);
        else    
            drivetrain.setDriveControlMode(DriveControlMode.JOYSTICKS);   
        if(setHold)
            drivetrain.setDriveControlMode(DriveControlMode.HOLD);      
        SmartDashboard.putNumber("Traveled BInches", Math.abs(drivetrain.getPose().translation.subtract(drivetrain.getBalanceInitialPos()).length));
        drivetrain.setBalanceInitialPos(Vector2.ZERO);
        drivetrain.setBrake();
        drivetrain.resetPose(new RigidTransform2(new Vector2(-164, drivetrain.getPose().translation.y), drivetrain.getPose().rotation));
        drivetrain.drive(Vector2.ZERO, 0.0, false);
    }
}
