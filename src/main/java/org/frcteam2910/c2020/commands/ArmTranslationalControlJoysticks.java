package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.subsystems.ArmExtender;
import org.frcteam2910.common.robot.input.Axis;

public class ArmTranslationalControlJoysticks extends CommandBase {
    private ArmExtender arm;
    private Axis YAxis;


    public ArmTranslationalControlJoysticks(ArmExtender arm, Axis YAxis) {
        this.YAxis = YAxis;
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        double speed = YAxis.get(true);
        if (Math.abs(speed) > 0.1) {
            arm.setTranslationalSpeed(speed*0.8);
        } else {
            arm.setTranslationalHold();
        }
    }
}
