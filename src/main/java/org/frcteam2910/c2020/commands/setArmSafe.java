package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.ScoreMode;

public class setArmSafe extends CommandBase {
    private final Arm arm;
    private final ScoreMode targetMode;
    private final ScoreMode startMode;
    private boolean goesAcross = false;
    private boolean wentIn = false;
    private boolean withinAngleTarget = false;
    private boolean withinTargetInches = false;
    private boolean wentOut = false;

    public setArmSafe(Arm arm, ScoreMode targetScoreMode) {
        this.arm = arm;
        this.targetMode = targetScoreMode;
        this.startMode = arm.getScoreMode();

        wentIn = false;
        withinAngleTarget = false;
        withinTargetInches = false;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if(arm.getScoreMode() != targetMode){
            switch(arm.getScoreMode()){
                case HIGH : 
                    if(targetMode==ScoreMode.ZERO || targetMode==ScoreMode.CONE_INTAKE || targetMode==ScoreMode.CUBE_INTAKE){
                        arm.setTargetArmInchesPositionAbsolute(0.0);
                        goesAcross = true;
                    } break;
                case MID :
                    if(targetMode==ScoreMode.ZERO || targetMode==ScoreMode.CONE_INTAKE || targetMode==ScoreMode.CUBE_INTAKE){
                        arm.setTargetArmInchesPositionAbsolute(0.0);
                        goesAcross = true;
                    } break;
                case CONE_INTAKE : 
                    arm.setArmDegreesPositionAbsolute(45.0);
                    while(!arm.withinAngle(5.0, 45.0)){}
                    arm.setTargetArmInchesPositionAbsolute(0.0);
                    goesAcross = true;
                    break;
                case CUBE_INTAKE : 
                    arm.setArmDegreesPositionAbsolute(45.0);
                    arm.setTargetArmInchesPositionAbsolute(0.0);
                    goesAcross = true;
                    break;        
                case ZERO : goesAcross = false; break;       
            }
            arm.setScoreMode(targetMode);
        }
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
