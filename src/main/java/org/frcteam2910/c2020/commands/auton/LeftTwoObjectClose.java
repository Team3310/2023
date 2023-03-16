package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class LeftTwoObjectClose extends AutonCommandBase {
    public LeftTwoObjectClose(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getIntake());
    }

    public LeftTwoObjectClose(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Intake intake) {
        boolean isBlue = getSide(container);
        resetRobotPose(container, trajectories.getThreeObjectClosePart1(isBlue));
        this.addCommands(
            new SetArmSafelyAuton(ScoreMode.HIGH),
            new SetIntakeRPM(intake, Constants.INTAKE_SPIT_RPM),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(()->intake.getConeSensor().get()),
                    new WaitCommand(0.3)
                ),
                new WaitCommand(1.0)
            ),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectClosePart1(isBlue)),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectClosePart2(isBlue)),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.geTthreeObjectCloseEnd1(isBlue))
        );
    }
}