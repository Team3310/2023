package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Arm;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetIntakeDeployPosition extends CommandBase {
    private final Intake intake;
    private boolean waitUntilReachedTarget = false;
    private double targetDegrees = 0.0;

    private final double DEPLOY_DEGREES_TOLERANCE = 5.0;

    public SetIntakeDeployPosition(Intake intake, double targetDegrees) {
        this(intake, targetDegrees, true);
    }

    public SetIntakeDeployPosition(Intake intake, double targetDegrees, boolean waitToFinishUntilTargetReached) {
        this.intake = intake;
        this.waitUntilReachedTarget = waitToFinishUntilTargetReached;
        this.targetDegrees = targetDegrees;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setCubeIntakeDeployTargetPosition(targetDegrees);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return intake.withinCubeDeployTarget(DEPLOY_DEGREES_TOLERANCE, targetDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        intake.clearDeployIntegrator();
    }
}
