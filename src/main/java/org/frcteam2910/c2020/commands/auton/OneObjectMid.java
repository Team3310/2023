package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class OneObjectMid extends AutonCommandBase {
    public OneObjectMid(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public OneObjectMid(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        resetRobotPose(container, trajectories.getOnToBridge());
        this.addCommands(
            //new ArmExtenderZero(Arm.getInstance()),
            new SetArmSafely(ScoreMode.CONE_HIGH),
            new WaitCommand(0.125),
            new SetArmIntakeRPM(intake, Constants.ARM_CONE_INTAKE_SPIT_RPM),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(()->!intake.getConeSensor().get()),
                    new WaitCommand(0.3)
                ),
                new WaitCommand(1.0)
            ),
            new ParallelCommandGroup(
                new SetArmSafely(ScoreMode.HOME),
                new SetArmIntakeRPM(intake, 0)
            )
        );
    }
}