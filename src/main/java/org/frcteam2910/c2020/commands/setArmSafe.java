package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import javax.xml.stream.events.EntityDeclaration;

import org.frcteam2910.c2020.subsystems.*;

public class setArmSafe extends CommandBase {
    private final ArmExtender extender;
    private final ArmRotator rotator;
    private final double angle;
    private final double inches;
    private boolean wentIn = false;
    private boolean extending = false;

    public setArmSafe(ArmExtender extender, ArmRotator rotator, double angle, double inches) {
        this.rotator = rotator;
        this.extender = extender;
        this.inches = inches;
        this.angle = angle;

        addRequirements(rotator);
        addRequirements(extender);
    }

    @Override
    public void initialize() {
        rotator.setArmDegreesPositionAbsolute(rotator.getArmDegrees()+10.0);
        extender.setTargetArmInchesPositionAbsolute(0.0);
    }

    @Override
    public void execute() {
        if(extender.getArmInches()<0.3){
            rotator.setArmDegreesPositionAbsolute(angle);
            wentIn=true;
        }
        if(wentIn){
            if(Math.abs(rotator.getArmDegrees())>10){
                extender.setTargetArmInchesPositionAbsolute(inches);
                extending = true;
            }
        }      
    }

    @Override
    public boolean isFinished(){
        return extending;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
