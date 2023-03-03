package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Intake;

public class VariableIntakeRPMCommand extends CommandBase {
    private Intake intake;


    public VariableIntakeRPMCommand(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        double inSpeed = intake.getIntakeAxis().get(true);
        double outSpeed = intake.getOuttakeAxis().get(true);

        if(inSpeed!=0)
            intake.setRollerRPM(Constants.INTAKE_COLLECT_RPM*inSpeed);
        else if(outSpeed!=0)    
            intake.setRollerRPM(Constants.INTAKE_SPIT_RPM*outSpeed);
        else
            intake.setRollerRPM(0);    
    }
}
