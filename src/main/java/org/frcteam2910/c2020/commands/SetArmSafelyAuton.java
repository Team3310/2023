package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.subsystems.Arm;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.util.ScoreMode;

public class SetArmSafelyAuton extends SequentialCommandGroup {
    private Arm arm;
    private final double DEGREES_DOWN = 8.5;

    public SetArmSafelyAuton(ScoreMode targetScoreMode, boolean afterIntake, boolean isCone) {
        this.arm = Arm.getInstance();
        arm.setRotationPIDSlot(targetScoreMode);

        this.addCommands(
            new InstantCommand(()->arm.setScoreMode(targetScoreMode))
        );

        addRequirements(arm);

        // this.addCommands(new PutString(startMode.name(), "start mode 1"));
        // this.addCommands(new PutString(targetScoreMode.name(), "target mode "));

        if(!afterIntake){
            if(targetScoreMode == ScoreMode.HOME){
                this.addCommands(
                    new SetArmExtender(arm, targetScoreMode.getInches(), true),
                    new SetArmRotator(arm, targetScoreMode.getAngle(), true)
                );
            }
            else if(targetScoreMode==ScoreMode.CUBE_INTAKE){
                this.addCommands(
                    // new InstantCommand(()->Intake.getInstance().setCubeIntakeDeployTargetPosition(111)),
                    // new InstantCommand(()->Intake.getInstance().setCubeRollerRPM(Constants.CUBE_INTAKE_ROLLER_HANDOFF_RPM)),
                    new SetArmExtender(arm, 0.0, true),
                    new SetArmRotator(arm, targetScoreMode.getAngle(), true),
                    new SetArmExtender(arm, targetScoreMode.getInches(), true)
                );
            }
            else{
                this.addCommands(
                    new SetArmExtender(arm, 0.0, true),
                    new SetArmRotator(arm, targetScoreMode.getAngle()-DEGREES_DOWN, true),
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
                    new SetIntakeDeployPosition(Intake.getInstance(), Constants.CUBE_INTAKE_DEPLOY_HOME_DEGREES), 
                    new InstantCommand(()->Intake.getInstance().setCubeRollerRPM(0)),
                    new SetArmExtender(arm, 0)
                );
            }
        }
    }
}    