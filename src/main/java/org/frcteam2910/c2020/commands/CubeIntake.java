package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CubeIntake extends SequentialCommandGroup{
    public CubeIntake(Intake intake, boolean handoff){
        if(handoff){
            this.addCommands(
                new SetArmSafely(ScoreMode.CUBE_INTAKE),
                new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES),
                new SetArmIntakeRPM(intake, Constants.ARM_CUBE_INTAKE_COLLECT_RPM),
                new InstantCommand(()->{
                    intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_COLLECT_RPM);
                    intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                    intake.stopRollingOnTriggeredArmIntakeDIO = true;
                })
            );
        }else{
            this.addCommands(
                new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES),
                new InstantCommand(()->{
                    intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_COLLECT_RPM);
                    intake.stopRollingOnTriggeredCubeIntakeDIO = true;
                    intake.stopRollingOnTriggeredArmIntakeDIO = true;
                })
            );
        }
    }
}
