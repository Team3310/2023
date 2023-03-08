package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import javax.xml.stream.events.EntityDeclaration;

import org.frcteam2910.c2020.subsystems.*;

public class setArm extends CommandBase {
    private final ArmExtender extender;
    private final ArmRotator rotator;
    private final double angle;
    private final double inches;

    public setArm(ArmExtender extender, ArmRotator rotator, double angle, double inches) {
        this.rotator = rotator;
        this.extender = extender;
        this.inches = inches;
        this.angle = angle;

        addRequirements(rotator);
        addRequirements(extender);
    }

    @Override
    public void initialize() {
        rotator.setArmDegreesPositionAbsolute(angle);
        extender.setTargetArmInchesPositionAbsolute(inches);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
