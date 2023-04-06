package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CubeSpit extends SequentialCommandGroup{
    public CubeSpit(Intake intake){
        this.addCommands(
            new InstantCommand(()->{
                intake.stopRollingOnTriggeredArmIntakeDIO = false;
                intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                intake.setCubeIntakeDeployTargetPosition(0);
                intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_SPIT_RPM*2, true);
                intake.setArmIntakeRPM(Constants.ARM_CUBE_INTAKE_SPIT_RPM*4, true);
            })
        );
    }
}
