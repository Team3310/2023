package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2020.util.ScoreMode;

public class SetArmSafelyAuton extends SequentialCommandGroup {
    public SetArmSafelyAuton(ScoreMode targetScoreMode, boolean afterIntake, boolean isCone){
        this.addCommands(
            new SetArmSafely(targetScoreMode, afterIntake, isCone),
            new WaitCommand(0.125)
        );
    } 
}    
