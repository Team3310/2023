package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.util.ScoreMode;
import org.frcteam2910.common.control.Trajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CubeIntakeAuton extends SequentialCommandGroup{
    public CubeIntakeAuton(Intake intake, boolean handoff, Trajectory trajectory){
        if(handoff){
            this.addCommands(
                new ParallelRaceGroup(
                    //wait for half the trajectory duration or until the arm gets to where its supposed to be
                    new SetArmSafely(ScoreMode.CUBE_INTAKE),
                    new WaitCommand(trajectory.getDuration()*0.5)    
                ),
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
