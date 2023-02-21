package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.ArmRotator;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;

public class ArmRotationControlJoysticks extends CommandBase {
    private ArmRotator arm;
    private Axis YAxis;


    public ArmRotationControlJoysticks(ArmRotator arm, Axis YAxis) {
        this.YAxis = YAxis;

        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        double speed = YAxis.get(true);

        if (Math.abs(speed) > 0.1) {
            arm.setRotationSpeed(speed);
        } else {
            arm.setRotationHold();
        }
    }
}
