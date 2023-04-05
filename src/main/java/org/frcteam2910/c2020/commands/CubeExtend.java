package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CubeExtend extends CommandBase {
    private final Arm arm;
    private boolean waitUntilReachedTarget = false;

    public CubeExtend(Arm arm, boolean waitToFinishUntilTargetReached) {
        this.arm = arm;
        this.waitUntilReachedTarget = waitToFinishUntilTargetReached;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.cubeExtend();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return (!waitUntilReachedTarget) || arm.withinCubeTarget();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
