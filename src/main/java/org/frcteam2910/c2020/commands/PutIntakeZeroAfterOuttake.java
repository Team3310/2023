package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.time.Instant;

import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.ScoreMode;

public class PutIntakeZeroAfterOuttake extends SequentialCommandGroup {
    private final Intake intake;
    private final Arm arm;

    public PutIntakeZeroAfterOuttake(Intake intake, Arm arm) {
        this.intake = intake;
        this.arm = arm;

        addRequirements(intake);
        addRequirements(arm);
        
        if(!intake.getConeSensor().get() && !intake.getCubeSensor().get()){
            this.addCommands(new setArmSafe(arm, ScoreMode.ZERO));
        }
        this.addCommands(new InstantCommand(()->intake.setRollerSpeed(0)));
    }
}
