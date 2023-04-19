package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Arm;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CubeIntake extends SequentialCommandGroup{
    public CubeIntake(Intake intake, boolean handoff){
        this(intake, handoff, false);
    }

    public CubeIntake(Intake intake, boolean handoff, boolean auton){
        if(handoff){
                if(auton){
                    this.addCommands(
                    new InstantCommand(()->{
                        intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_COLLECT_RPM, true);
                        intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                        intake.stopRollingOnTriggeredArmIntakeDIO = true;
                    }),
                    new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES),
                    new ParallelRaceGroup(
                        new SequentialCommandGroup(
                            new SetArmExtender(Arm.getInstance(), 0, true),
                            new SetArmRotator(Arm.getInstance(), ScoreMode.CUBE_INTAKE.getAngle()+4),
                            new SetArmExtender(Arm.getInstance(), ScoreMode.CUBE_INTAKE.getInches())
                        ),
                        new WaitCommand(1.0)    
                    ),
                    new SetArmIntakeRPM(intake, Constants.ARM_CUBE_INTAKE_COLLECT_RPM, true)
                );
            }
            else{
                this.addCommands(
                    new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES),
                    new ParallelRaceGroup(
                        new SetArmSafely(ScoreMode.CUBE_INTAKE),
                        new WaitCommand(1.0)    
                    ),
                    new SetArmIntakeRPM(intake, Constants.ARM_CUBE_INTAKE_COLLECT_RPM, true),
                    new InstantCommand(()->{
                        intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_COLLECT_RPM, true);
                        intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                        intake.stopRollingOnTriggeredArmIntakeDIO = true;
                    })
                );
            }
        }else{
            this.addCommands(
                new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES),
                new InstantCommand(()->{
                    intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_COLLECT_RPM, true);
                    intake.stopRollingOnTriggeredCubeIntakeDIO = true;
                    intake.stopRollingOnTriggeredArmIntakeDIO = true;
                })
            );
        }
    }    
}
