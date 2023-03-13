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
            new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPlaceToLoad()),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getConeBridgeLoadTOPickup1()),
                new SequentialCommandGroup(
                    new setArmSafe(ScoreMode.CONE_INTAKE),
                    new InstantCommand(()->{
                        intake.setRollerRPM(Constants.INTAKE_COLLECT_RPM); 
                        intake.setServoPosition(-1.0);
                    })
                )
            ),
            new ParallelRaceGroup( 
                //waits until cone sensor says it has a cone or 1.5 seconds has elasped in case it doesn't get cone   
                new WaitUntilCommand(new BooleanSupplier() {
                    @Override
                    public boolean getAsBoolean() {
                        return intake.getConeSensor().get();
                    }  
                }),
                new WaitCommand(1.5)
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPickUp1ToLoad()),
                new SequentialCommandGroup(
                    new setArmSafe(ScoreMode.ZERO),
                    new InstantCommand(()->{
                        intake.setRollerSpeed(0);
                        intake.setServoPosition(1.0);
                    })
                )
            ),  
            new ParallelDeadlineGroup(  
                new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromLoadtoPlace()),
                new setArmSafe(ScoreMode.MID)
            ),    
            new ParallelDeadlineGroup(  
                new ParallelRaceGroup( 
                  //waits until is out or 1.0 seconds has elasped in case   
                  new SequentialCommandGroup(   
                    new WaitUntilCommand(new BooleanSupplier() {
                        @Override
                        public boolean getAsBoolean() {
                            return intake.getConeSensor().get();
                        }  
                    }),
                    new WaitCommand(0.25) //TODO test timing for object outtake
                ),
                    new WaitCommand(1.0)
                ),
                new InstantCommand(()->intake.setRollerRPM(Constants.INTAKE_SPIT_RPM))
            ),
            new ParallelDeadlineGroup(      
                new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPlaceToLoad()),
                new SequentialCommandGroup(
                    new setArmSafe(ScoreMode.ZERO),
                    new InstantCommand(()->{
                        intake.setRollerSpeed(0);
                    })   
                )
            )    
        );
    }
}