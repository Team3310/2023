package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetIntakeRPM extends CommandBase {
    private final Intake intake;
    private final double rpm;

    public SetIntakeRPM(Intake intake, double rpm) {
        this.intake = intake;
        this.rpm = rpm;
    }

    @Override
    public void execute() {
        intake.setArmIntakeRPM(rpm);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
