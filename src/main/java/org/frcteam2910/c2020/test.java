package org.frcteam2910.c2020;

import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.SideChooser.SideMode;
import org.frcteam2910.common.control.Trajectory;

public class test {
    static AutonomousTrajectories test = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, SideMode.BLUE);

    public static void main(String[] args){
        Trajectory traj = test.getThreeObjectFarPart2(false);
        for(double time = 0.0; time<traj.getDuration(); time+=(traj.getDuration()/10)){
            System.out.println("rotation : " + traj.calculate(time).getPathState().getRotation());
        } 
    }
}
