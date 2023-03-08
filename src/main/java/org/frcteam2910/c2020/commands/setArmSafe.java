package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.subsystems.*;

public class setArmSafe extends CommandBase {
    private final ArmExtender extender;
    private final ArmRotator rotator;
    private final double targetAngle;
    private final double targetInches;
    private boolean wentIn = false;
    private boolean atTargetAngle = false;
    private boolean atTargetInches = false;
    private double degreesTravelled = 0.0;
    private final double startDegrees;
    private final double startInches;
    private final boolean needsToCross;

    public setArmSafe(ArmExtender extender, ArmRotator rotator, double targetAngle, double targetInches) {
        this.rotator = rotator;
        this.extender = extender;
        this.targetInches = targetInches;
        this.targetAngle = targetAngle;
        this.startDegrees = rotator.getArmDegrees();
        this.startInches = extender.getArmInches();

        needsToCross = targetAngle>0?
                        rotator.getArmDegrees()>0?false:true:
                        rotator.getArmDegrees()<0?false:true;
        SmartDashboard.putNumber("check needs to cross number", Math.pow(targetAngle, rotator.getArmDegrees()));
        SmartDashboard.putBoolean("needs to cross", needsToCross);

        addRequirements(rotator);
        addRequirements(extender);
    }

    @Override
    public void initialize() {
        if(needsToCross){
            if(Math.abs(startDegrees)<20 && Math.abs(startDegrees) > 0.5){
                if(startDegrees>0)
                    rotator.setArmDegreesPositionAbsolute(rotator.getArmDegrees()+10.0);
                else
                    rotator.setArmDegreesPositionAbsolute(rotator.getArmDegrees()-10.0);    
            }
            if(startInches > 0.3){
                extender.setTargetArmInchesPositionAbsolute(0.0);
            } 
        }
        else{
            atTargetAngle = true;
            extender.setTargetArmInchesPositionAbsolute(targetInches);
            rotator.setArmDegreesPositionAbsolute(targetAngle);
        }   
    }

    @Override
    public void execute() {
        if(true){
            if(extender.getArmInches()>0.3){
                extender.setTargetArmInchesPositionAbsolute(0.0);
            }
            else{
                wentIn = true;
            }
            if(wentIn){
                if(!atTargetAngle){
                    rotator.setArmDegreesPositionAbsolute(targetAngle);
                }
                else if(!atTargetInches){
                    extender.setArmInchesZero(targetInches);
                }
                atTargetInches = Math.abs(extender.getArmInches()-targetInches) < 0.5;
                atTargetAngle = Math.abs(Math.abs(rotator.getArmDegrees())-targetAngle) < 0.5;
            }
        }
        SmartDashboard.putBoolean("at target inches", atTargetInches);
        SmartDashboard.putBoolean("at target angle", atTargetAngle);
    }

    @Override
    public boolean isFinished(){
        return atTargetAngle && atTargetInches;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
