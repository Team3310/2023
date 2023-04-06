package org.frcteam2910.c2020;

import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.Intake;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.util.SideChooser.SideMode;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.control.PathSegment;

public class test {
    static AutonomousTrajectories test = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, SideMode.BLUE);

    public static void main(String[] args){
        Intake test = Intake.getInstance();
        System.out.println(test.ArmRoller_RpmToVelocityTicks(Constants.ARM_CONE_INTAKE_SPIT_RPM));
    }

    public static void printCoords(Trajectory trajectory, Trajectory trajectory2){
        if(true){
            for(int i=0; i<trajectory.getPath().getSegments().length; i++){
                PathSegment segment = trajectory.getPath().getSegments()[i];
                PathSegment segment2 = trajectory2.getPath().getSegments()[i];
                System.out.println("start x:"+segment.getStart().getPosition().x + " : "+ segment2.getStart().getPosition().x);
                System.out.println("end x:"+segment.getEnd().getPosition().x+ " : "+ segment2.getEnd().getPosition().x);
            }
        }
    }
}
