package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class SetUpForTeleop extends ParallelCommandGroup{
    public SetUpForTeleop(Intake intake){
        this.addCommands(
            // new SetArmSafely(ScoreMode.HOME),
            // new SetArmIntakeRPM(intake, 0),
            new InstantCommand(()->{
                    intake.setCubeRollerRPM(0, true);
                    intake.setCubeIntakeDeployTargetPosition(Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES);
                    intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                    intake.stopRollingOnTriggeredArmIntakeDIO = false;
            })
        );
    }
}
