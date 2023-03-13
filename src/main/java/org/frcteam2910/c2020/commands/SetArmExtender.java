package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmExtender extends CommandBase {
    private final Arm arm;
    private boolean waitUntilReachedTarget = false;
    private boolean withinToleranceOfTarget = false;
    private double targetInches = Double.MIN_VALUE;

    private final double ARM_INCHES_TOLERANCE = 0.5;

    public SetArmExtender(Arm arm, double targetInches) {
        this(arm, targetInches, true);
    }

    public SetArmExtender(Arm arm, double targetInches, boolean waitToFinishUntilTargetReached) {
        this.arm = arm;
        this.waitUntilReachedTarget = waitToFinishUntilTargetReached;
        this.withinToleranceOfTarget = false;
        this.targetInches = targetInches;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setTargetArmInchesPositionAbsolute(targetInches);
    }

    @Override
    public void execute() {
        if(!withinToleranceOfTarget) {
            withinToleranceOfTarget = arm.withinInches(ARM_INCHES_TOLERANCE, targetInches);
        }
    }

    @Override
    public boolean isFinished(){
        return (!waitUntilReachedTarget) || withinToleranceOfTarget;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
