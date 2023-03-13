package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frcteam2910.c2020.subsystems.Arm;
import org.frcteam2910.c2020.util.ScoreMode;

public class setArmSafe extends SequentialCommandGroup {
    private final Arm arm;
    private final ScoreMode targetMode;
    private final ScoreMode startMode;
    private boolean wasUnsafeManeuver = false;

    public setArmSafe(Arm arm, ScoreMode targetScoreMode) {
        this.arm = arm;
        this.targetMode = targetScoreMode;
        this.startMode = arm.getScoreMode();
        wasUnsafeManeuver = true;

        addRequirements(arm);

        if(startMode != targetMode){
            switch(startMode){
                case HIGH :
                case MID :
                    // We check the targetMode == LOW here because we could hit the high/mid scoring positions with the extension if we don't.
                    if(targetMode==ScoreMode.ZERO || targetMode==ScoreMode.CONE_INTAKE || targetMode==ScoreMode.CUBE_INTAKE || targetMode==ScoreMode.LOW){
                        // We must SAFELY move to the above positions -- to do this we must retract
                        this.addCommands(new SetArmExtender(arm, 0.0, true));
                        wasUnsafeManeuver = true;
                    } break;
                case LOW :
                    if(targetMode==ScoreMode.ZERO || targetMode==ScoreMode.CUBE_INTAKE || targetMode==ScoreMode.CONE_INTAKE){
                        // We must SAFELY move to the above positions -- to do this we must retract
                        this.addCommands(new SetArmExtender(arm, 0.0, true));
                        wasUnsafeManeuver = true;
                    } break;
                case CONE_INTAKE : 
                case CUBE_INTAKE : 
                    if(targetMode == ScoreMode.ZERO) {
                        // If going to Zero position after intaking from the front, bring the object up then in
                        this.addCommands(new SetArmRotator(arm, 45.0, true));
                        this.addCommands(new SetArmExtender(arm, 0.0, true));
                        wasUnsafeManeuver = false;
                    } break;
                case ZERO : 
                    // We started from zero; the assumption here is that we're fully retracted and have properly zeroed the rotator.
                    wasUnsafeManeuver = false;
                    break;       
            }
            arm.setScoreMode(targetMode);
        }

        SmartDashboard.putBoolean("SetArm Retract?", wasUnsafeManeuver);
        // Tell the arm to sequentially move to the target angle, then extend
        this.addCommands(new SetArmRotator(arm, targetMode.getAngle(), true));
        this.addCommands(new SetArmExtender(arm, targetMode.getInches(), true));
    }
}
