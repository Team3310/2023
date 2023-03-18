package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.SideChooser.SideMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PlaceConeAndLeave extends AutonCommandBase {
    public PlaceConeAndLeave(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public PlaceConeAndLeave(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive) {
        boolean isBlue = RobotContainer.getInstance().getSideChooser().getSendableChooser().getSelected() == SideMode.BLUE;
        resetRobotPose(container, trajectories.placeAndLeave(isBlue));
        this.addCommands(
            new OneObjectMid(container, trajectories),
            new InstantCommand(()->drive.setBridgeDriveVoltage(3)),
            new ChangeDriveMode(drive, DriveControlMode.BRIDGE_VOLTAGE),
            new WaitCommand(1.5),
            new InstantCommand(()->drive.setBridgeDriveVoltage(0))
            //new FollowTrajectoryCommand(drive, trajectories.placeAndLeave(isBlue)) 
        );
    }
}