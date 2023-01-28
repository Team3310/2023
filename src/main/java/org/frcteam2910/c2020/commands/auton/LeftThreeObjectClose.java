package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LeftThreeObjectClose extends AutonCommandBase {
    public LeftThreeObjectClose(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public LeftThreeObjectClose(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive) {
        resetRobotPose(container, trajectories.getThreeObjectClosePart1());
        this.addCommands(
            //path with commands on old robot
            /*new FollowTrajectoryCommand(drive, trajectories.getTestPart1()),
            new ParallelDeadlineGroup(
                new WaitCommand(1.0),
                new EjectBallsAuton(intake, indexer, shooter)
            ),
            new IntakeIndexerHalt(intake, indexer),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getTestPart2()),
                new SequentialCommandGroup(
                    new WaitCommand(1.0),
                    new IntakeSetRPM(intake, 500.0)
                )
            ),
            new WaitCommand(1.0),
            new IntakeIndexerHalt(intake, indexer),
            new FollowTrajectoryCommand(drive, trajectories.getTestPart3()),
            new ParallelDeadlineGroup(
                new WaitCommand(1.0),
                new EjectBallsAuton(intake, indexer, shooter)
            ),
            new IntakeIndexerHalt(intake, indexer),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getTestPart4()),
                new SequentialCommandGroup(
                    new WaitCommand(2.0),
                    new IntakeSetRPM(intake, 750.0)
                )
            ),    
            new WaitCommand(0.75),
            new IntakeIndexerHalt(intake, indexer),
            new FollowTrajectoryCommand(drive, trajectories.getTestPart5()),
            new ParallelDeadlineGroup(
                new WaitCommand(1.0),
                new EjectBallsAuton(intake, indexer, shooter)
            ),
            new IntakeIndexerHalt(intake, indexer)*/
            //just path
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectClosePart1()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectClosePart2()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectClosePart3()),
            new WaitCommand(0.5),
            new FollowTrajectoryCommand(drive, trajectories.getThreeObjectClosePart4()),
            new WaitCommand(0.5)
            //new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart5())
        );
    }
}