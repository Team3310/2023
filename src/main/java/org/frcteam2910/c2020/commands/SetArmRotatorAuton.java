package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

@Deprecated
public class SetArmRotatorAuton extends CommandBase {
    private final Arm arm;
    private boolean waitUntilReachedTarget = false;
    private double targetDegrees = Double.MIN_VALUE;

    private final double ARM_DEGREES_TOLERANCE = 5.0;

    public SetArmRotatorAuton(Arm arm, double targetDegrees) {
        this(arm, targetDegrees, true);
    }

    public SetArmRotatorAuton(Arm arm, double targetDegrees, boolean waitToFinishUntilTargetReached) {
        this.arm = arm;
        this.waitUntilReachedTarget = waitToFinishUntilTargetReached;
        this.targetDegrees = targetDegrees;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setArmDegreesPositionAbsolute(targetDegrees);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return (!waitUntilReachedTarget) || arm.withinAngle(ARM_DEGREES_TOLERANCE, targetDegrees);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
