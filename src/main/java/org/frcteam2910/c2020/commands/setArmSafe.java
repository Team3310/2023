package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.subsystems.*;

public class setArmSafe extends CommandBase {
    private final ArmExtender extender;
    private final ArmRotator rotator;
    private final double targetAngle;
    private final double targetInches;
    private final double startAngle;
    private boolean goesAcross;
    private boolean wentIn = false;
    private boolean withinAngleTarget = false;
    private boolean withinTargetInches = false;

    public setArmSafe(ArmExtender extender, ArmRotator rotator, double targetAngle, double targetInches) {
        this.rotator = rotator;
        this.extender = extender;
        this.targetInches = targetInches;
        this.targetAngle = targetAngle;
        this.startAngle = rotator.getArmDegrees();

        wentIn = false;
        withinAngleTarget = false;
        withinTargetInches = false;

        goesAcross = (startAngle>=0 && targetAngle<0) || (startAngle<=0 && targetAngle>0) || targetAngle == 0;

        addRequirements(rotator);
        addRequirements(extender);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("target angle from command", targetAngle);
        if(Math.abs(startAngle)<=20){
            rotator.setArmDegreesPositionAbsolute(startAngle+Math.copySign(10,startAngle));
        }
        if(goesAcross){
            extender.setTargetArmInchesPositionAbsolute(0.0);
        }
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("target angle from command2", targetAngle);
        SmartDashboard.putBoolean("goes across", goesAcross);
        SmartDashboard.putBoolean("within target angle", withinAngleTarget);
        SmartDashboard.putBoolean("went in", wentIn);
        SmartDashboard.putBoolean("within target inches", withinTargetInches);
        if(goesAcross){
            wentIn = extender.getArmInches()<0.5;
            withinAngleTarget = rotator.withinAngle(5.0, targetAngle);
            if(wentIn){
                SmartDashboard.putNumber("targetAngle 6", targetAngle);
                rotator.setArmDegreesPositionAbsolute(targetAngle);
                if(withinAngleTarget){
                    extender.setTargetArmInchesPositionAbsolute(targetInches);
                    withinTargetInches = extender.withinInches(0.5, targetInches);
                }
            }
        }
        else{
            SmartDashboard.putNumber("targetAngle 7", targetAngle);
            rotator.setArmDegreesPositionAbsolute(targetAngle);
            withinAngleTarget = rotator.withinAngle(5.0, targetAngle);
            if(withinAngleTarget){
                extender.setTargetArmInchesPositionAbsolute(targetInches);
                withinTargetInches = extender.withinInches(0.5, targetInches);
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
