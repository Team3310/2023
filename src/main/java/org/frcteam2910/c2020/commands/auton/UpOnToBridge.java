package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Vector2;

public class UpOnToBridge extends AutonCommandBase {
    public UpOnToBridge(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public UpOnToBridge(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive){
        // Trajectory upOnToBridge = trajectories.getUpBridge(new Vector2(drive.getPose().translation.x, drive.getPose().translation.y), drive.getPose().rotation);
        Trajectory upOnToBridge = trajectories.getUpBridge(drive.getPose().translation, drive.getPose().rotation);
        resetRobotPose(container, upOnToBridge);
        this.addCommands(
            new FollowTrajectoryCommand(drive, upOnToBridge)
        );
    }
}