package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmExtender extends CommandBase {
    private final Arm arm;
    private boolean waitUntilReachedTarget = false;
    private double targetInches = Double.MIN_VALUE;

    private final double ARM_INCHES_TOLERANCE = 1.5;

    public SetArmExtender(Arm arm, double targetInches) {
        this(arm, targetInches, true);
    }

    public SetArmExtender(Arm arm, double targetInches, boolean waitToFinishUntilTargetReached) {
        this.arm = arm;
        this.waitUntilReachedTarget = waitToFinishUntilTargetReached;
        this.targetInches = targetInches;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setTargetArmInchesPositionAbsolute(targetInches);
        SmartDashboard.putBoolean("Finished Ext", false);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return (!waitUntilReachedTarget) || arm.withinInches(ARM_INCHES_TOLERANCE, targetInches);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Finished Ext", true);
    }
}
