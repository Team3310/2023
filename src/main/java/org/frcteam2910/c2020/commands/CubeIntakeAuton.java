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
                new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES).
                andThen(
                    new ParallelRaceGroup(
                        //wait for at least 5 seconds or the trajectory duration(if<5) or until the arm gets to where its supposed to be
                        //shouldn't need to worry about arm intake rolling into cube lift intake but just in case
                        new SetArmSafely(ScoreMode.CUBE_INTAKE),
                        new WaitCommand(2.5)
                        //new WaitCommand(trajectory.getDuration()<10?trajectory.getDuration()<5?trajectory.getDuration():5:trajectory.getDuration()*0.5)    
                    )
                ).
                andThen(
                    new SetArmIntakeRPM(intake, Constants.ARM_CUBE_INTAKE_COLLECT_RPM, true)
                ).andThen(
                    new InstantCommand(()->{
                        intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_COLLECT_RPM, true);
                        intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                        intake.stopRollingOnTriggeredArmIntakeDIO = true;
                    })
                )
            );
        }else{
            this.addCommands(
                new SequentialCommandGroup(
                    new SetIntakeDeployPosition(intake, Constants.CUBE_INTAKE_DEPLOY_MAX_DEGREES).
                    andThen(
                        new InstantCommand(()->{
                            intake.setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_COLLECT_RPM, true);
                            intake.stopRollingOnTriggeredCubeIntakeDIO = true;
                            intake.stopRollingOnTriggeredArmIntakeDIO = true;
                        })
                    )
                )
            );
        }
    }
}
