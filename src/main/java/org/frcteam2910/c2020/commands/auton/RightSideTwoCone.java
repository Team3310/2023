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

public class RightSideTwoCone extends AutonCommandBase {
    public RightSideTwoCone(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem(), container.getArm(), container.getIntake());
    }

    public RightSideTwoCone(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive, Arm arm, Intake intake) {
        this.addCommands(
            new RightSideOneCone(container, trajectories),
            new ParallelDeadlineGroup(     
                new FollowTrajectoryCommand(drive, trajectories.getConeBridgeLoadToPickup2()),
                new SetArmSafe(ScoreMode.CONE_INTAKE, false),
                new InstantCommand(()->{
                    intake.setRollerRPM(Constants.INTAKE_COLLECT_RPM); 
                    intake.setServoPosition(-1.0);
                })
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
                new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromPickUp2ToLoad()),
                new SequentialCommandGroup(
                    new SetArmSafe(ScoreMode.ZERO, false),
                    new InstantCommand(()->{
                        intake.setRollerSpeed(0);
                        intake.setServoPosition(1.0);
                    })
                )
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromLoadtoPlace()),
                new SetArmSafe(ScoreMode.HIGH, false)
            ),
            new ParallelDeadlineGroup(  
                new ParallelRaceGroup( 
                  //waits until cone is out or 1.0 seconds has elasped in case
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
            )
        );
    }
}