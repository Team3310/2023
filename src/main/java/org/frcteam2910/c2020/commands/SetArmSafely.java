package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Arm;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.util.ScoreMode;

public class SetArmSafely extends SequentialCommandGroup {
    private Arm arm;

    public SetArmSafely(ScoreMode targetScoreMode){
        this(targetScoreMode, false, false);
    }

    public SetArmSafely(ScoreMode targetScoreMode, boolean afterIntake){
        this(targetScoreMode, afterIntake, false);
    }

    public SetArmSafely(boolean afterIntake, boolean isCone){
        this(ScoreMode.HOME, afterIntake, isCone);
    }

    public SetArmSafely(ScoreMode targetScoreMode, boolean afterIntake, boolean isCone) {

        this.arm = Arm.getInstance();
        Intake intake = Intake.getInstance();
        // arm.setScoreMode(!afterIntake?targetScoreMode:ScoreMode.ZERO);
        this.addCommands(
            new InstantCommand(()->{
                arm.clearRotatorIWindup();
                arm.setScoreMode(!afterIntake?targetScoreMode:ScoreMode.HOME);
                arm.setRotationPIDSlot(targetScoreMode);
            })
        );

        addRequirements(arm);

        if(!afterIntake){
            if(targetScoreMode!=ScoreMode.CUBE_INTAKE || targetScoreMode==ScoreMode.HOME){
                this.addCommands(
                    new SetArmExtender(arm, 0.0, true),
                    new SetArmRotator(arm, targetScoreMode.getAngle(), true),
                    new SetArmExtender(arm, targetScoreMode.getInches(), true)
                );
            }
            else if(targetScoreMode==ScoreMode.CUBE_INTAKE){
                this.addCommands(
                    // new InstantCommand(()->Intake.getInstance().setCubeIntakeDeployTargetPosition(111)),
                    new SetArmExtender(arm, 0.0, true),
                    new SetArmRotator(arm, targetScoreMode.getAngle(), true),
                    new SetArmExtender(arm, targetScoreMode.getInches(), true),
                    new InstantCommand(()->Intake.getInstance().setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_HANDOFF_RPM, true))
                );
            }
        }else{
            if(isCone){
                this.addCommands(
                    new SetArmExtender(arm, 5.5, true),
                    new SetArmRotator(arm, 35.0, true),
                    new SetArmExtender(arm, 0, true),
                    new SetArmRotator(arm, 10, true)    
                );
            }
            else{
                this.addCommands(
                    new SetIntakeDeployPosition(Intake.getInstance(), Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES), 
                    new SetArmRotator(arm, ScoreMode.HOME.getAngle()),
                    new InstantCommand(()->{
                        intake.stopRollingOnTriggeredCubeIntakeDIO = false;
                        intake.stopRollingOnTriggeredArmIntakeDIO = false;
                        intake.resetIntakeDIOTimestamp();
                        if(!intake.setIntakeHold){
                            intake.setCubeRollerRPM(0, true);
                            intake.setArmIntakeRPM(0, true);
                        }
                    })
                );
            }
        }
    }
}    