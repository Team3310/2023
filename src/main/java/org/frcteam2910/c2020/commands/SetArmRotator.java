package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Arm;

import org.frcteam2910.c2020.util.CheckMovement;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmRotator extends CommandBase {
    private final Arm arm;
    private boolean waitUntilReachedTarget = false;
    private double targetDegrees = Double.MIN_VALUE;
    private CheckMovement moveChecker;

    private final double ARM_DEGREES_TOLERANCE = 5.0;

    public SetArmRotator(Arm arm, double targetDegrees) {
        this(arm, targetDegrees, true);
    }

    public SetArmRotator(Arm arm, double targetDegrees, boolean waitToFinishUntilTargetReached) {
        this.arm = arm;
        this.waitUntilReachedTarget = waitToFinishUntilTargetReached;
        this.targetDegrees = targetDegrees;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        this.moveChecker = new CheckMovement(arm.getArmDegreesIntegrated(), 0.5, 2.0);
        arm.setArmDegreesPositionAbsolute(targetDegrees);
    }

    @Override
    public void execute() {
        if(moveChecker.check(arm.getArmDegreesIntegrated())){
            this.end(true); //just set to true for use if we want to do something when it ends due to no movement
        }
    }

    @Override
    public boolean isFinished(){
        // if((!waitUntilReachedTarget) || arm.withinAngle(ARM_DEGREES_TOLERANCE, targetDegrees))
        //     arm.setScoreMode(ScoreMode.getClosestMode(arm.getArmDegreesIntegrated()));
        return ((!waitUntilReachedTarget) || arm.withinAngle(ARM_DEGREES_TOLERANCE, targetDegrees));
    }

    @Override
    public void end(boolean interrupted) {
    }
}