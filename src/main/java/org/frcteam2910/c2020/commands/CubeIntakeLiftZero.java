package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class CubeIntakeLiftZero extends CommandBase {
    private double MIN_LIFT_DEGREES_CHANGE = 0.5;
    private Timer timer = new Timer();
    private final Intake intake;
    private double lastPosition;

    public CubeIntakeLiftZero(Intake intake) {
        this.intake = intake;
        
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setCubeLiftSpeed(-Constants.CUBE_INTAKE_LIFT_ZERO_SPEED);
        lastPosition = Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES;
        timer.start();
    }

    @Override
    public void execute() {
        intake.setCubeLiftSpeed(-Constants.CUBE_INTAKE_LIFT_ZERO_SPEED);
    }

    @Override
    public boolean isFinished() {
        double currentPosition = intake.getCubeIntakeDeployDegrees();
        double positionChange = currentPosition - lastPosition;
        lastPosition = currentPosition;
        boolean haveMoved = Math.abs(positionChange) >= MIN_LIFT_DEGREES_CHANGE;
        if(haveMoved) {
            // We're counting seconds we haven't moved
            timer.reset();
            timer.start();
        }
        return timer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setCubeIntakeDeployZeroReference(Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES);
        intake.setCubeIntakeDeployTargetPosition(0.0);
    }
}