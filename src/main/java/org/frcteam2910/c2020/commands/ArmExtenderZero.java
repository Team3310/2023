package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.ArmExtender;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ArmExtenderZero extends CommandBase {
    private double MIN_ARM_EXTEND_POSITION_CHANGE = 0.5;
    private Timer timer = new Timer();
    private final ArmExtender armExtender;
    private double lastPosition;

    public ArmExtenderZero(ArmExtender armExtender) {
        this.armExtender = armExtender;
        
        addRequirements(armExtender);
    }

    @Override
    public void initialize() {
        armExtender.setTranslationalSpeed(-Constants.ARM_EXTEND_ZEROING_SPEED, true);
        lastPosition = Constants.ARM_MAX_EXTEND_INCHES;
        timer.start();
    }

    @Override
    public void execute() {
        armExtender.setTranslationalSpeed(-Constants.ARM_EXTEND_ZEROING_SPEED, true);
    }

    @Override
    public boolean isFinished() {
        double currentPosition = armExtender.getArmInches();
        double positionChange = currentPosition - lastPosition;
        lastPosition = currentPosition;
        boolean haveMoved = Math.abs(positionChange) >= MIN_ARM_EXTEND_POSITION_CHANGE;
        SmartDashboard.putBoolean("Done", timer.hasElapsed(1));
        SmartDashboard.putNumber("Change", positionChange);
        SmartDashboard.putNumber("Timer", timer.get());
        if(haveMoved) {
            // We're counting seconds we haven't moved
            timer.reset();
            timer.start();
        }
        return timer.hasElapsed(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        armExtender.setArmInchesZero(Constants.ARM_EXTEND_HOME_INCHES);
        armExtender.setExtenderHold();
    }
}