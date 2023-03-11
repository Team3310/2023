package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.subsystems.Arm;
import org.frcteam2910.common.robot.input.Axis;

public class ArmJoystickControl extends CommandBase {
    private Arm arm;
    private Axis extenderAxis;
    private Axis rotationAxis;


    public ArmJoystickControl(Arm arm, Axis extenderAxis, Axis rotationAxis) {
        this.extenderAxis = extenderAxis;
        this.rotationAxis = rotationAxis;
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        double extenderSpeed = extenderAxis.get(true);
        if (Math.abs(extenderSpeed) > 0.1) {
            arm.setTranslationalSpeed(extenderSpeed*0.8);
        } else {
            arm.setTranslationalHold();
        }  

        double rotationSpeed = rotationAxis.get(true);
        if (Math.abs(rotationSpeed) > 0.1) {
            arm.setRotationSpeed(rotationSpeed / 2.0);
        } else {
            arm.setRotationHold();
        }
    }
}
