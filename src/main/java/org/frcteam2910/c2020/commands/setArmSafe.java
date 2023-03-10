package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.*;

public class setArmSafe extends CommandBase {
    private final ArmExtender extender;
    private final ArmRotator rotator;
    private final double targetAngle;
    private final double targetInches;
    private boolean setIn = false;
    private final double startAngle;
    private final boolean needsToCross;

    public setArmSafe(ArmExtender extender, ArmRotator rotator, double targetAngle, double targetInches) {
        this.rotator = rotator;
        this.extender = extender;
        this.targetInches = targetInches;
        this.targetAngle = targetAngle;
        this.startAngle = rotator.getArmDegrees();

        if(targetAngle>0){
            if(startAngle <= 0){
                needsToCross = true;
            }
            else if(startAngle > 0){
                needsToCross = false;
            }
            else{
                needsToCross = true; //TODO simplify logic
            }
        }
        else if(targetAngle<0){
            if(startAngle >= 0){
                needsToCross = true;
            }
            else if(startAngle<0){
                needsToCross = false;
            }
            else{
                needsToCross = true; //set defualt to true just to be safe
            }
        }
        else{
            needsToCross = true;
        }

        addRequirements(rotator);
        addRequirements(extender);
    }

    @Override
    public void initialize() {
        if(needsToCross){
            if(Math.abs(startAngle)<=25){
                rotator.setArmDegreesPositionAbsolute(Math.copySign(Math.abs(startAngle)+10, startAngle));
            }
            extender.setTargetArmInchesPositionAbsolute(0);
            setIn = true;
        }
        else{
            if(Math.abs(startAngle)<=25){
                rotator.setArmDegreesPositionAbsolute(Math.copySign(Math.abs(startAngle)+10, startAngle));
                if(Math.abs(targetAngle)<Math.abs(startAngle)){
                    extender.setTargetArmInchesPositionAbsolute(0);
                    setIn = true;
                }
            } 
        }   
    }

    @Override
    public void execute() {
        if(setIn){
            if(extender.getArmAtZero()){
                rotator.setArmDegreesPositionAbsolute(targetAngle);
            }
            if(rotator.getArmWithinTarget(20)){
                extender.setTargetArmInchesPositionAbsolute(targetInches);
            }
        }
        else{
            extender.setTargetArmInchesPositionAbsolute(targetInches);
            rotator.setArmDegreesPositionAbsolute(startAngle);
        }
    }

    @Override
    public boolean isFinished(){
        return extender.getArmWithinTarget(0.5) && rotator.getArmWithinTarget(0.5);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
