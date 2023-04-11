package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmIntakeRPM extends CommandBase {
    private final Intake intake;
    private final double rpm;
    private final boolean override;

    public SetArmIntakeRPM(Intake intake, double rpm, boolean override) {
        this.intake = intake;
        this.rpm = rpm;
        this.override = override;
    }

    @Override
    public void execute() {
        intake.setArmIntakeRPM(rpm, override);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
