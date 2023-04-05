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
            new SetArmIntakeRPM(intake, -Constants.ARM_CONE_INTAKE_SPIT_RPM)
            // new ParallelDeadlineGroup(     
            //     new FollowTrajectoryCommand(drive, trajectories.getConeBridgeToPickup2(isBlue)),
            //     new SetArmSafely(ScoreMode.CONE_INTAKE, false),
            //     new InstantCommand(()->{
            //         intake.setRollerRPM(Constants.INTAKE_COLLECT_RPM); 
            //         intake.setServoPosition(-1.0);
            //     })
            // ),
            // new ParallelRaceGroup( 
            //     //waits until cone sensor says it has a cone or 1.5 seconds has elasped in case it doesn't get cone   
            //     new WaitUntilCommand(new BooleanSupplier() {
            //         @Override
            //         public boolean getAsBoolean() {
            //             return intake.getConeSensor().get();
            //         }  
            //     }),
            //     new WaitCommand(1.5)
            // ),
            // new ParallelDeadlineGroup(
            //     new FollowTrajectoryCommand(drive, trajectories.getConeBridgeToPlace2(isBlue)),
            //     new SequentialCommandGroup(
            //         new SetArmSafely(ScoreMode.ZERO, false),
            //         new InstantCommand(()->{
            //             intake.setRollerSpeed(0);
            //             intake.setServoPosition(1.0);
            //         })
            //     )
            // ),
            // new ParallelDeadlineGroup(
            //     new FollowTrajectoryCommand(drive, trajectories.getConeBridgefromLoadtoPlace(isBlue)),
            //     new SetArmSafely(ScoreMode.HIGH, false)
            // ),
            // new ParallelDeadlineGroup(  
            //     new ParallelRaceGroup( 
            //       //waits until cone is out or 1.0 seconds has elasped in case
            //         new SequentialCommandGroup(   
            //             new WaitUntilCommand(new BooleanSupplier() {
            //                 @Override
            //                 public boolean getAsBoolean() {
            //                     return intake.getConeSensor().get();
            //                 }  
            //             }),
            //             new WaitCommand(0.25) //TODO test timing for object outtake
            //         ),
            //         new WaitCommand(1.0)
            //     ),
            //     new InstantCommand(()->intake.setRollerRPM(Constants.INTAKE_SPIT_RPM))
            // )
        );
    }
}