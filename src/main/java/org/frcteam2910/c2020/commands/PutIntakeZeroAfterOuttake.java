package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Arm;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PutIntakeZeroAfterOuttake extends SequentialCommandGroup {
    private final Intake intake;
    private final Arm arm;

    public PutIntakeZeroAfterOuttake(Intake intake, Arm arm) {
        this.intake = intake;
        this.arm = arm;

        addRequirements(intake);
        addRequirements(arm);
        
        this.addCommands(new InstantCommand(()->intake.setRollerSpeed(0)));
        if(!intake.getConeSensor().get() && !intake.getCubeSensor().get()){
            // We no longer possess an object (and are likely in a LOW/MID/HIGH ScoreMode), return to zero
            this.addCommands(new SetArmSafely(ScoreMode.ZERO, false));
        }
    }
}
