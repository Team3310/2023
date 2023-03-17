package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ArmExtenderZero extends CommandBase {
    private double MIN_ARM_EXTEND_POSITION_CHANGE = 0.05;
    private Timer timer = new Timer();
    private final Arm arm;
    private double lastPosition;

    public ArmExtenderZero(Arm arm) {
        this.arm = arm;
        
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setTranslationalSpeed(-Constants.ARM_EXTEND_ZEROING_SPEED, true);
        lastPosition = Constants.ARM_MAX_EXTEND_INCHES;
        timer.start();
    }

    @Override
    public void execute() {
        arm.setTranslationalSpeed(-Constants.ARM_EXTEND_ZEROING_SPEED, true);
    }

    @Override
    public boolean isFinished() {
        double currentPosition = arm.getArmInches();
        double positionChange = currentPosition - lastPosition;
        lastPosition = currentPosition;
        boolean haveMoved = Math.abs(positionChange) >= MIN_ARM_EXTEND_POSITION_CHANGE;
        SmartDashboard.putBoolean("Done", timer.hasElapsed(0.25));
        SmartDashboard.putNumber("Change", positionChange);
        SmartDashboard.putNumber("Timer", timer.get());
        if(haveMoved) {
            // We're counting seconds we haven't moved
            timer.reset();
            timer.start();
        }
        return timer.hasElapsed(0.25);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmInchesZero(Constants.ARM_EXTEND_HOME_INCHES);
        arm.setTranslationalHold();
    }
}