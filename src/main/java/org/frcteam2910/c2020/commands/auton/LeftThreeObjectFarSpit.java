package org.frcteam2910.c2020.commands.auton;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

public class LeftThreeObjectFarSpit extends AutonCommandBase {
    public LeftThreeObjectFarSpit(RobotContainer container, AutonomousTrajectories trajectories){
        this(container, trajectories, container.getDrivetrainSubsystem());
    }

    public LeftThreeObjectFarSpit(RobotContainer container, AutonomousTrajectories trajectories, DrivetrainSubsystem drive) {
        boolean isBlue = getSide(container);
        resetRobotPose(container, trajectories.getThreeObjectFarPart1(false));
        this.addCommands(
            new SetArmSafely(ScoreMode.CONE_MID),
            new SetIntakeRPM(Intake.getInstance(), Constants.ARM_INTAKE_SPIT_RPM),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(()->!Intake.getInstance().getConeSensor().get()),
                    new WaitCommand(0.3)
                ),
                new WaitCommand(0.75)
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart1(false)),
                new SequentialCommandGroup(
                    new SetIntakeDeployPosition(Intake.getInstance(), 110),
                    new SetIntakeRPM(Intake.getInstance(), 0)
                ),
                new SetArmSafely(ScoreMode.HOME),
                new InstantCommand(()->{
                    Intake.getInstance().setCubeRollerRPM(2000);
                    Intake.getInstance().stopRollingOnTriggeredCubeIntakeDIO = true;
                }) 
            ),
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart2(false)),
                new InstantCommand(()->Intake.getInstance().setCubeRollerRPM(0)),
                new SetIntakeDeployPosition(Intake.getInstance(), 110)
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                new InstantCommand(()->Intake.getInstance().setCubeRollerRPM(-2000))
            ),
            new ParallelDeadlineGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart3(false)),
                new SetArmSafely(ScoreMode.CUBE_INTAKE),
                new SetIntakeDeployPosition(Intake.getInstance(), 110),
                new SetIntakeRPM(Intake.getInstance(), Constants.ARM_CUBE_INTAKE_COLLECT_RPM),
                new InstantCommand(()->{
                    Intake.getInstance().setCubeRollerRPM(2000);
                    Intake.getInstance().stopRollingOnTriggeredCubeIntakeDIO = false;
                    Intake.getInstance().stopRollingOnTriggeredArmIntakeDIO = true;
                })
            ),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new FollowTrajectoryCommand(drive, trajectories.getThreeObjectFarPart4B(false)),
                new SequentialCommandGroup(
                    new SetArmSafely(ScoreMode.HOME),
                    new WaitCommand(0.6),
                    new SetArmSafely(ScoreMode.CUBE_MID)
                ),  
                new SetIntakeDeployPosition(Intake.getInstance(), 0),
                new SetIntakeRPM(Intake.getInstance(), 0),
                new InstantCommand(()->{
                    Intake.getInstance().setCubeRollerRPM(0);
                    Intake.getInstance().stopRollingOnTriggeredCubeIntakeDIO = false;
                    Intake.getInstance().stopRollingOnTriggeredArmIntakeDIO = false;
                })
            ),
            new SetIntakeRPM(Intake.getInstance(), Constants.ARM_CUBE_INTAKE_SPIT_RPM),
            new WaitCommand(0.3),
            new SetArmSafely(ScoreMode.HOME),
            new SetIntakeRPM(Intake.getInstance(), 0)
        );
    }
}