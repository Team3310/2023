package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.BooleanSupplier;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.ScoreMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class RightSideOneCone extends AutonCommandBase {
    public RightSideOneCone(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public RightSideOneCone(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        resetRobotPose(container, trajectories.getConeBridgefromPlaceToLoad());
        this.addCommands(
            new SetArmSafelyAuton(ScoreMode.HIGH),
            new SetIntakeRPM(intake, Constants.INTAKE_SPIT_RPM),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(()->!intake.getConeSensor().get()),
                    new WaitCommand(0.3)
                ),
                new WaitCommand(1.0)
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPlaceToLoad()),
                new SetIntakeRPM(intake, 0),
                new SetArmSafelyAuton(ScoreMode.ZERO)
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getConeBridgeLoadTOPickup1()),
                new SetArmSafelyAuton(ScoreMode.CONE_INTAKE),
                new SetIntakeRPM(intake, Constants.INTAKE_COLLECT_RPM),
                new SetServosOut(intake)
            ),
            new ParallelRaceGroup( 
                //waits until cone sensor says it has a cone or 1.5 seconds has elasped in case it doesn't get cone   
                new WaitUntilCommand(()->intake.getConeSensor().get()),
                new WaitCommand(0.75)
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPickUp1ToLoad()),
                new SetArmSafely(ScoreMode.ZERO),
                new SetIntakeRPM(intake, 0),
                new SetServosIn(intake)
            ),  
            new ParallelDeadlineGroup(  
                new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromLoadtoPlace()),
                new SetArmSafelyAuton(ScoreMode.MID)
            ),    
            new ParallelDeadlineGroup(  
                new ParallelRaceGroup( 
                  //waits until is out or 1.0 seconds has elasped in case   
                  new SequentialCommandGroup(   
                    new WaitUntilCommand(()->!intake.getConeSensor().get()),
                    new WaitCommand(0.25) //TODO test timing for object outtake
                ),
                    new WaitCommand(1.0)
                ),
                new SetIntakeRPM(intake, Constants.INTAKE_SPIT_RPM)
            ),
            new ParallelCommandGroup(
                new SetIntakeRPM(intake, 0),
                new SetArmSafelyAuton(ScoreMode.ZERO) 
            )  
        );
    }
}