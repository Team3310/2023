package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.common.robot.input.Axis;

public class VariableIntakeRPMCommand extends CommandBase {
    private Intake intake;
    private Axis inAxis;
    private Axis outAxis;


    public VariableIntakeRPMCommand(Intake arm, Axis YAxis, Axis outAxis) {
        this.inAxis = YAxis;
        this.outAxis = outAxis;
        this.intake = arm;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        double inSpeed = inAxis.get(true);
        double outSpeed = outAxis.get(true);

        if(inSpeed!=0)
            intake.setRollerRPM(Constants.INTAKE_COLLECT_RPM*inSpeed);
        else if(outSpeed!=0)    
            intake.setRollerRPM(Constants.INTAKE_SPIT_RPM*outSpeed);
        else
            intake.setRollerRPM(0);    
    }
}
