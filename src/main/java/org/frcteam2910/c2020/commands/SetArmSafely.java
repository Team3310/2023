package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.time.Instant;
import java.util.function.BooleanSupplier;

import org.frcteam2910.c2020.subsystems.Arm;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.util.ScoreMode;

public class SetArmSafely extends SequentialCommandGroup {

    private Arm arm;
    // private final ScoreMode targetScoreMode;
    private final ScoreMode startMode;
    private boolean wasUnsafeManeuver = false;

    public SetArmSafely(ScoreMode targetScoreMode){
        this(targetScoreMode, false, false);
    }

    public SetArmSafely(ScoreMode targetScoreMode, boolean afterIntake){
        this(targetScoreMode, afterIntake, false);
    }

    public SetArmSafely(boolean afterIntake, boolean isCone){
        this(afterIntake?null:ScoreMode.ZERO, afterIntake, isCone);
    }

    public SetArmSafely(ScoreMode targetScoreMode, boolean afterIntake, boolean isCone) {

        // SmartDashboard.putString("target score mode", targetScoreMode.name());
        // SmartDashboard.putString("new score mode", arm.getScoreMode().name());

        this.arm = Arm.getInstance();
        // this.targetScoreMode = targetScoreMode;
        this.startMode = arm.getScoreMode();
        wasUnsafeManeuver = true;

        // arm.setScoreMode(!afterIntake?targetScoreMode:ScoreMode.ZERO);
        this.addCommands(
            new InstantCommand(()->arm.setScoreMode(!afterIntake?targetScoreMode:ScoreMode.ZERO))
        );

        addRequirements(arm);

        // this.addCommands(new PutString(startMode.name(), "start mode 1"));
        // this.addCommands(new PutString(targetScoreMode.name(), "target mode "));

        if(!afterIntake){
            if(targetScoreMode!=ScoreMode.CUBE_INTAKE){
                this.addCommands(
                    new SetArmExtender(arm, 0.0, true),
                    new SetArmRotator(arm, targetScoreMode.getAngle(), true),
                    new SetArmExtender(arm, targetScoreMode.getInches(), true)
                );
            }
            else if(targetScoreMode==ScoreMode.CUBE_INTAKE){
                this.addCommands(
                    // new InstantCommand(()->Intake.getInstance().setCubeIntakeDeployTargetPosition(111)),  
                    new InstantCommand(()->Intake.getInstance().setCubeRollerRPM(800)),
                    new SetArmExtender(arm, 0.0, true),
                    new SetArmRotator(arm, targetScoreMode.getAngle(), true),
                    new SetArmExtender(arm, targetScoreMode.getInches(), true)
                );
            }
        }else{
            if(isCone){
                this.addCommands(
                    new SetArmExtender(arm, 4.5, true),
                    new SetArmRotator(arm, 35.0, true),
                    new SetArmExtender(arm, 0, true),
                    new SetArmRotator(arm, 10, true)    
                );
            }
            else{
                this.addCommands(
                    new SetIntakeDeployPosition(Intake.getInstance(), 0), 
                    new InstantCommand(()->Intake.getInstance().setCubeRollerRPM(0)),
                    new SetArmExtender(arm, 0)
                );
            }
        }

        // if(startMode != targetScoreMode || (!arm.withinAngle(5.0, targetScoreMode.getAngle()) || !arm.withinInches(0.5, targetScoreMode.getInches()))){
        //     switch(startMode !=targetScoreMode?ScoreMode.getClosestMode(arm.getArmDegrees()):startMode){
        //         case HIGH :
        //         case MID :
        //             // We check the targetMode == LOW here because we could hit the high/mid scoring positions with the extension if we don't.
        //             if(targetScoreMode==ScoreMode.ZERO || targetScoreMode==ScoreMode.CONE_INTAKE || targetScoreMode==ScoreMode.CUBE_INTAKE || targetScoreMode==ScoreMode.LOW){
        //                 // We must SAFELY move to the above positions -- to do this we must retract
        //                 this.addCommands(new SetArmExtender(arm, 0.0, true));
        //                 wasUnsafeManeuver = true;
        //             } break;
        //         case LOW :
        //             if(targetScoreMode==ScoreMode.ZERO || targetScoreMode==ScoreMode.CUBE_INTAKE || targetScoreMode==ScoreMode.CONE_INTAKE){
        //                 // We must SAFELY move to the above positions -- to do this we must retract
        //                 this.addCommands(new SetArmExtender(arm, 0.0, true));
        //                 wasUnsafeManeuver = true;
        //             } break;
        //         case CONE_INTAKE : 
        //         case CUBE_INTAKE : 
        //             if(targetScoreMode == ScoreMode.ZERO) {
        //                 // If going to Zero position after intaking from the front, bring the object up then in
        //                 this.addCommands(new SetArmRotator(arm, 45.0, true));
        //                 this.addCommands(new SetArmExtender(arm, 0.0, true));
        //                 wasUnsafeManeuver = false;
        //             } break;
        //         case ZERO : 
        //             // We started from zero; the assumption here is that we're fully retracted and have properly zeroed the rotator.
        //             wasUnsafeManeuver = false;
        //             break;       
        //     }

        //     // SmartDashboard.putBoolean("SetArm Retract?", wasUnsafeManeuver);
        //     // Tell the arm to sequentially move to the target angle, then extend
        //     this.addCommands(new SetArmRotator(arm, targetScoreMode.getAngle(), true));
        //     this.addCommands(new SetArmExtender(arm, targetScoreMode.getInches(), true));
        // }
    }
}    