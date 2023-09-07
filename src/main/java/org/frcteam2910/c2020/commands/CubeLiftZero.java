package org.frcteam2910.c2020.commands;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class CubeLiftZero extends CommandBase {
    private double MIN_POSITION_CHANGE = 1.0;
    private Timer timer = new Timer();
    private final Intake intake;
    private double lastPosition;

    public CubeLiftZero(Intake intake) {
        this.intake = intake;
        
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setCubeLiftSpeed(-Constants.CUBE_LIFT_ZEROING_SPEED);
        lastPosition = Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES;
        timer.start();
    }

    @Override
    public void execute() {
        System.out.println("moving");
        intake.setCubeLiftSpeed(-Constants.CUBE_LIFT_ZEROING_SPEED);
        SmartDashboard.putBoolean("ended lift zero", false);
    }

    @Override
    public boolean isFinished() {
        double currentPosition = intake.getCubeIntakeDeployDegrees();
        double positionChange = currentPosition - lastPosition;
        lastPosition = currentPosition;
        boolean haveMoved = Math.abs(positionChange) >= MIN_POSITION_CHANGE;
        // SmartDashboard.putBoolean("Done", timer.hasElapsed(0.25));
        // SmartDashboard.putNumber("Change", positionChange);
        // SmartDashboard.putNumber("Timer", timer.get());
        if(haveMoved) {
            // We're counting seconds we haven't moved
            timer.reset();
            timer.start();
        }
        return timer.hasElapsed(0.25);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("interrupted: "+interrupted);
        intake.setCubeIntakeDeployZeroReference(Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES);
        intake.setCubeIntakeDeployTargetPosition(Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES);
        SmartDashboard.putBoolean("ended life zero", true);
    }
}