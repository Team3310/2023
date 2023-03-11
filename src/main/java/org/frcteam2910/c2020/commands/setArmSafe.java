package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.ScoreMode;

public class setArmSafe extends CommandBase {
    private final Arm arm;
    private final ScoreMode targetMode;
    private boolean goesAcross = false;
    private boolean wentIn = false;
    private boolean withinAngleTarget = false;
    private boolean withinTargetInches = false;

    public setArmSafe(Arm arm, ScoreMode targetScoreMode) {
        this.arm = arm;
        this.targetMode = targetScoreMode;

        wentIn = false;
        withinAngleTarget = false;
        withinTargetInches = false;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        switch(arm.getScoreMode()){
            case HIGH : 
                if(targetMode==ScoreMode.ZERO || targetMode==ScoreMode.INTAKE){
                    arm.setTargetArmInchesPositionAbsolute(0.0);
                    goesAcross = true;
                } break;
            case MID :
                if(targetMode==ScoreMode.ZERO || targetMode==ScoreMode.INTAKE){
                    arm.setTargetArmInchesPositionAbsolute(0.0);
                    goesAcross = true;
                } break;
            case INTAKE : 
                    arm.setArmDegreesPositionAbsolute(arm.getArmDegrees()+10);
                    arm.setTargetArmInchesPositionAbsolute(0.0);
                    goesAcross = true;
                    break;
            case ZERO : goesAcross = false; break;       
        }
        arm.setScoreMode(targetMode);
    }

    @Override
    public void execute() {
        if(goesAcross){
            wentIn = arm.withinInches(0.5, 0.0);
            withinTargetInches = arm.withinInches(0.5, targetMode.getInches());
            withinAngleTarget = arm.withinAngle(5.0, targetMode.getAngle());

            if(wentIn){
                arm.setArmDegreesPositionAbsolute(targetMode.getAngle());
                if(withinAngleTarget){
                    arm.setTargetArmInchesPositionAbsolute(targetMode.getInches());
                }
            }
        }
        else{
            withinTargetInches = arm.withinInches(0.5, targetMode.getInches());
            withinAngleTarget = arm.withinAngle(5.0, targetMode.getAngle());

            arm.setArmDegreesPositionAbsolute(targetMode.getAngle());
            if(withinAngleTarget){
                arm.setTargetArmInchesPositionAbsolute(targetMode.getInches());
            }
        }
    }

    @Override
    public boolean isFinished(){
        return withinTargetInches && withinAngleTarget;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
