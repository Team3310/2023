package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;

public class ArmTranslationalControlJoysticks extends CommandBase {
    private Intake intake;
    private Axis YAxis;


    public ArmTranslationalControlJoysticks(Intake intake, Axis YAxis) {
        this.YAxis = YAxis;

        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        double speed = YAxis.get(true);

        if (Math.abs(speed) > 0.1) {
            intake.setTranslationalSpeed(speed);
        } else {
            intake.setTranslationalHold();
        }
    }
}
