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

public class OneObjectMidTest extends AutonCommandBase {
    public OneObjectMidTest(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public OneObjectMidTest(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        resetRobotPose(container, trajectories.getOnToBridge());
        this.addCommands(
            //new ArmExtenderZero(Arm.getInstance()),
            new SetArmSafelyAuton(ScoreMode.CONE_MID, false, false),
            new ScoreConeAuton(intake),
            new ParallelCommandGroup(
                new SetArmSafely(ScoreMode.HOME),
                new SetArmIntakeRPM(intake, 0, true)
            )
        );
    }
}