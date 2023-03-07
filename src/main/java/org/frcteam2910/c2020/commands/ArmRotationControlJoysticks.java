package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.subsystems.ArmRotator;

public class ArmRotationControlJoysticks extends CommandBase {
    private ArmRotator arm;

    public ArmRotationControlJoysticks(ArmRotator arm) {
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        double speed = arm.getArmRotationAxis().get(true);

        if (Math.abs(speed) > 0.1) {
            arm.setRotationSpeed(speed / 2.0);
        } else {
            arm.setRotationHold();
        }
    }
}
