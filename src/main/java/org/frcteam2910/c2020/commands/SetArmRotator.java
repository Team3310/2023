package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmRotator extends CommandBase {
    private final Arm arm;
    private boolean waitUntilReachedTarget = false;
    private boolean withinToleranceOfTarget = false;
    private double startingDegrees = Double.MIN_VALUE;
    private double targetDegrees = Double.MIN_VALUE;

    private final double ARM_DEGREES_TOLERANCE = 2.0;

    public SetArmRotator(Arm arm, double targetDegrees) {
        this(arm, targetDegrees, true);
    }

    public SetArmRotator(Arm arm, double targetDegrees, boolean waitToFinishUntilTargetReached) {
        this.arm = arm;
        this.waitUntilReachedTarget = waitToFinishUntilTargetReached;
        this.withinToleranceOfTarget = false;
        this.startingDegrees = arm.getArmDegrees();
        this.targetDegrees = targetDegrees;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setArmDegreesPositionAbsolute(targetDegrees);
    }

    @Override
    public void execute() {
        if(!withinToleranceOfTarget) {
            withinToleranceOfTarget = arm.withinAngle(ARM_DEGREES_TOLERANCE, targetDegrees);
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
