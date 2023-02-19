package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.*;

public class SetServosOut extends CommandBase {
    private final Intake intake;

    public SetServosOut(Intake drivetrain) {
        this.intake = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        intake.setServoSpeed(1.0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
