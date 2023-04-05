package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ScoreCubeAuton extends SequentialCommandGroup{
    public ScoreCubeAuton(Intake intake){
        this.addCommands(
            new SetArmIntakeRPM(intake, Constants.ARM_CUBE_INTAKE_SPIT_RPM, true),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(()->!intake.getCubeSensor().get()),
                    new WaitCommand(0.2)
                ),
                new WaitCommand(0.75)
            )
        );
    }
}