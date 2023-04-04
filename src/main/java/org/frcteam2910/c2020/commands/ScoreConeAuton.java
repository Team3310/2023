package org.frcteam2910.c2020.commands;

import org.frcteam2910.c2020.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreConeAuton extends SequentialCommandGroup{
    public ScoreConeAuton(Intake intake){
        this.addCommands(
            new SetArmIntakeRPM(intake, Constants.ARM_CONE_INTAKE_SPIT_RPM),
            new SetArmIntakeRPM(intake, Constants.ARM_CONE_INTAKE_SPIT_RPM),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(()->!intake.getConeSensor().get()),
                    new WaitCommand(0.2)
                ),
                new WaitCommand(0.75)
            )
        );
    }
}