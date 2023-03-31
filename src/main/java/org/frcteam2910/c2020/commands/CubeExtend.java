package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CubeExtend extends CommandBase {
    private final Arm arm;
    private boolean waitUntilReachedTarget = false;

    private final double ARM_INCHES_TOLERANCE = 1.5;

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
        return (!waitUntilReachedTarget) || arm.withinTarget();
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Finished Ext", true);
    }
}
