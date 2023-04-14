package org.frcteam2910.c2020.commands.auton;


import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RightSide2HalfBalance extends AutonCommandBase {
    public RightSide2HalfBalance(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public RightSide2HalfBalance(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        boolean isBlue = false;//getSide(container);
        this.addCommands(
            new EasySideTwoObject(container, trajectories),
            new FollowTrajectoryCommand(drive, trajectories.getEasySideConeToPickup2(isBlue)),
            new WaitCommand(1.0),
            new FollowTrajectoryCommand(drive, trajectories.getEasySideToBridge2Half()),
            new WaitCommand(0.2),
            new OnToBridge(container, trajectories, 5)
        );
    }
}