package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RightSideThreeCone extends AutonCommandBase {
    public RightSideThreeCone(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public RightSideThreeCone(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        boolean isBlue = false;//getSide(container);
        this.addCommands(
            new RightSideTwoCone(container, trajectories),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getEasySideConeToPickup2(isBlue)),
                new SetArmSafely(ScoreMode.CUBE_INTAKE, false, false) 
            ),
            new WaitCommand(1.0),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getEasySideConeToPlace2(isBlue)),
                new SetArmSafely(true, false) 
            ),
            new SetArmSafely(ScoreMode.CONE_MID),
            new SetArmIntakeRPM(intake, -Constants.ARM_CONE_INTAKE_SPIT_RPM, true)
        );
    }
}