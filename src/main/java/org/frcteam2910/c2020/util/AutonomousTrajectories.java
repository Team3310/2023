package org.frcteam2910.c2020.util;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Arrays;

import org.frcteam2910.c2020.util.SideChooser.SideMode;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.SimplePathBuilder;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.control.TrajectoryConstraint;
import org.frcteam2910.common.io.PathReader;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;
    private int xReflect;
    private double angleOffset;

//#region Trajectories
    private Trajectory sevenFeet;
    private final Trajectory sCurve;

    private Trajectory threeObjectFarPart1;
    private Trajectory threeObjectFarPart2;
    private Trajectory threeObjectFarPart3;
    private Trajectory threeObjectFarPart4;
    private Trajectory threeObjectFarPart5;

    private Trajectory threeObjectClosePart1;
    private Trajectory threeObjectClosePart2;
    private Trajectory threeObjectClosePart3;
    private Trajectory threeObjectClosePart4;
    private Trajectory threeObjectCloseEnd1;
    private Trajectory threeObjectCloseEnd2;

    private Trajectory threeObjectBridgePart1;
    private Trajectory threeObjectBridgePart2;
    private Trajectory threeObjectBridgePart3;
    private Trajectory threeObjectBridgePart4;

    private Trajectory coneBridgeToPickUp1;
    private Trajectory coneBridgeToPickUp2;
    private Trajectory coneBridgeToPlace1;
    private Trajectory coneBridgeToPlace2;

    private Trajectory onToBridge;
    private Trajectory goPastBridge;
    private Trajectory getToConePastBridge;

    private Trajectory threeObjectFarPart1Blue;
    private Trajectory threeObjectFarPart2Blue;
    private Trajectory threeObjectFarPart3Blue;
    private Trajectory threeObjectFarPart4Blue;
    private Trajectory threeObjectFarPart5Blue;

    private Trajectory threeObjectClosePart1Blue;
    private Trajectory threeObjectClosePart2Blue;
    private Trajectory threeObjectClosePart3Blue;
    private Trajectory threeObjectClosePart4Blue;
    private Trajectory threeObjectCloseEnd1Blue;
    private Trajectory threeObjectCloseEnd2Blue;

    private Trajectory threeObjectBridgePart1Blue;
    private Trajectory threeObjectBridgePart2Blue;
    private Trajectory threeObjectBridgePart3Blue;
    private Trajectory threeObjectBridgePart4Blue;

    private Trajectory coneBridgeToPickUp1Blue;
    private Trajectory coneBridgeToPickUp2Blue;
    private Trajectory coneBridgeToPlace1Blue;
    private Trajectory coneBridgeToPlace2Blue;
    private Trajectory coneBridgeToBridge1;
    private Trajectory coneBridgeToBridge1Blue;
    private Trajectory coneBridgeToBridge2;
    private Trajectory coneBridgeToBridge2Half;
//#endregion

    private TrajectoryConstraint[] bridgeConstraints;
    private TrajectoryConstraint[] mediumConstraints;
    private TrajectoryConstraint[] slowConstraints;
    private TrajectoryConstraint[] bumpConstraints;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints, SideMode side){
        slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);
        
        mediumConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        mediumConstraints[mediumConstraints.length - 1] = new MaxVelocityConstraint(16.0 * 12.0); //9
        mediumConstraints[mediumConstraints.length - 2] = new MaxAccelerationConstraint(7 * 12.0);//4

        bridgeConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        bridgeConstraints[mediumConstraints.length - 1] = new MaxVelocityConstraint(2.0 * 12.0);
        bridgeConstraints[mediumConstraints.length - 2] = new MaxAccelerationConstraint(2.0 * 12.0);

        bumpConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        bumpConstraints[mediumConstraints.length - 1] = new MaxVelocityConstraint(2.0 * 12.0);
        bumpConstraints[mediumConstraints.length - 2] = new MaxAccelerationConstraint(1.0 * 12.0);

        if(side==SideMode.RED){
            xReflect=-1;      
        }else{
            xReflect=1;
            angleOffset=180;
        }

        sCurve = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(60, 0), Rotation2.ZERO)
                        .arcTo(new Vector2(100, 40), new Vector2(60, 40))
                        .lineTo(new Vector2(100, 60), Rotation2.ZERO)
                        .arcTo(new Vector2(140, 100), new Vector2(140, 60), Rotation2.ZERO)
                        .lineTo(new Vector2(200, 100))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

//#region Three Object Far
        threeObjectFarPart1 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*156.5,298), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*254.5, 298))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart2 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*255.5,298), Rotation2.fromDegrees(0+angleOffset))//71,298 non spit
                        .lineTo(new Vector2(xReflect*67.5, 276))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart3 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*67.5, 278), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*254.5, 268))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart4 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*254.5, 268), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*122.5, 273))
                        //.lineTo(new Vector2(259, 228))
                        .arcTo(new Vector2(xReflect*67.5, 228), new Vector2(xReflect*122.5, 217.5))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart5 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*67.5, 228), Rotation2.fromDegrees(0+angleOffset))
                        .arcTo(new Vector2(xReflect*122.5, 273), new Vector2(xReflect*122.5, 217.6))
                        .lineTo(new Vector2(xReflect*220.5, 273))
                        .arcTo(new Vector2(xReflect*252.5, 228), new Vector2(xReflect*121, 230))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectFarPart1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*156.5,298), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*254.5, 298))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart2Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*255.5,298), Rotation2.fromDegrees(0+angleOffset))//71,298 non spit
                        .lineTo(new Vector2(xReflect*67.5, 276))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart3Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*67.5, 278), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*254.5, 268))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart4Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*254.5, 268), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*122.5, 273))
                        //.lineTo(new Vector2(259, 228))
                        .arcTo(new Vector2(xReflect*67.5, 228), new Vector2(xReflect*122.5, 217.5))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart5Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*67.5, 228), Rotation2.fromDegrees(0+angleOffset))
                        .arcTo(new Vector2(xReflect*122.5, 273), new Vector2(xReflect*122.5, 217.6))
                        .lineTo(new Vector2(xReflect*220.5, 273))
                        .arcTo(new Vector2(xReflect*252.5, 228), new Vector2(xReflect*121, 230))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
//#endregion
//#region Three Object Close
        threeObjectClosePart1 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*259.5, 297.5), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*58.75, 279))
                        .build(),
                bumpConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart2 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*68.75, 279), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*259.5, 297.5))
                        .build(),
                bumpConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart3 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*259.5, 297.5), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*135, 286))
                        .lineTo(new Vector2(xReflect*58.5, 230.5))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart4 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*58.5, 230.5), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*135, 286))
                        .lineTo(new Vector2(xReflect*227.6, 279))
                        .lineTo(new Vector2(xReflect*258, 250))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectCloseEnd1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*259.5, 297.5), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*237.9, 230))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        threeObjectCloseEnd2Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*258, 260), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*237.9, 230))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*259.5, 297.5), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*58.75, 279))
                        .build(),
                bumpConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart2Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*68.75, 279), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*259.5, 297.5))
                        .build(),
                bumpConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart3Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*259.5, 297.5), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*135, 286))
                        .lineTo(new Vector2(xReflect*58.5, 230.5))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart4Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*58.5, 230.5), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*135, 286))
                        .lineTo(new Vector2(xReflect*227.6, 279))
                        .lineTo(new Vector2(xReflect*258, 250))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectCloseEnd1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*259.5, 297.5), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*237.9, 230))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        threeObjectCloseEnd2Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*258, 260), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*237.9, 230))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
//#endregion
//#region Three Object Bridge
        threeObjectBridgePart1 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*254.5, 161.5), Rotation2.fromDegrees(0+angleOffset))
                        .arcTo(new Vector2(xReflect*238.5, 128), new Vector2(xReflect*235.5, 158))
                        .lineTo(new Vector2(xReflect*66, 128))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart2 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*66, 128), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*251.5, 138))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart3 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*251.5, 138), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*126.5, 130))
                        .arcTo(new Vector2(xReflect*56.5, 182), new Vector2(xReflect*126.5, 201))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart4 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*56.5, 182), Rotation2.fromDegrees(0+angleOffset))
                        .arcTo(new Vector2(xReflect*129.5, 131), new Vector2(xReflect*135.5, 201.5))
                        .lineTo(new Vector2(xReflect*251.5, 126))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*254.5, 161.5), Rotation2.fromDegrees(0+angleOffset))
                        .arcTo(new Vector2(xReflect*238.5, 128), new Vector2(xReflect*235.5, 158))
                        .lineTo(new Vector2(xReflect*66, 128))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart2Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*66, 128), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*251.5, 138))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart3Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*251.5, 138), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*126.5, 130))
                        .arcTo(new Vector2(xReflect*56.5, 182), new Vector2(xReflect*126.5, 201))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart4Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*56.5, 182), Rotation2.fromDegrees(0+angleOffset))
                        .arcTo(new Vector2(xReflect*129.5, 131), new Vector2(xReflect*135.5, 201.5))
                        .lineTo(new Vector2(xReflect*251.5, 126))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
//#endregion
//#region Cone Bridge
        coneBridgeToPickUp2 =
                new Trajectory(new SimplePathBuilder(new Vector2(-246, 140.68), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(-123.3, 140.68), Rotation2.fromDegrees(225))
                        .arcTo(new Vector2(-28.95271996, 195.00), new Vector2(-123.29914997, 246.99918908)).build(),
                        mediumConstraints, SAMPLE_DISTANCE);

        coneBridgeToPlace2 = new Trajectory(
                new SimplePathBuilder(new Vector2(-28.95271996, 195.00), Rotation2.fromDegrees(225))
                        .arcTo(new Vector2(-123.3, 140.68), new Vector2(-123.29914997, 246.99918908))
                        .lineTo(new Vector2(-250, 146.68), Rotation2.fromDegrees(160))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE);


        coneBridgeToPickUp1 = new Trajectory(
                new SimplePathBuilder(new Vector2(-250, 119), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(-32.1, 140.5))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE);        

        coneBridgeToPlace1 = new Trajectory(
                new SimplePathBuilder(new Vector2(-32.1, 140.5), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(-246, 140.68))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE);        
        coneBridgeToBridge1 = new Trajectory(
                new SimplePathBuilder(new Vector2(-246, 140.68), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(-240, 140.68))
                        .lineTo(new Vector2(-240, 197.68))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );
        //#region blue
        coneBridgeToPickUp2Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(246, 140.68), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(123.3, 140.68), Rotation2.fromDegrees(45))
                        .arcTo(new Vector2(28.95271996, 195.00), new Vector2(123.29914997, 246.99918908))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        coneBridgeToPlace2Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(28.95271996, 195.00), Rotation2.fromDegrees(45))
                        .arcTo(new Vector2(123.3, 140.68), new Vector2(123.29914997, 246.99918908))
                        .lineTo(new Vector2(250, 146.68), Rotation2.fromDegrees(0))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        coneBridgeToPickUp1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(250, 119), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(32.1, 140.5))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        coneBridgeToPlace1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(32.1, 140.5), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(246, 140.68))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );
        coneBridgeToBridge1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(246, 140.68), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(240, 140.68))
                        .lineTo(new Vector2(240, 197.68))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );
        //#endregion
//#endregion        
        onToBridge = new Trajectory(
                new SimplePathBuilder(new Vector2(-252, 207), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(-173, 207), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(-176, 207), Rotation2.fromDegrees(180))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE);

        goPastBridge = new Trajectory(
                new SimplePathBuilder(new Vector2(-176, 207), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(-156, 207), Rotation2.fromDegrees(0))
                        .build(),
                bridgeConstraints, SAMPLE_DISTANCE);

        getToConePastBridge = new Trajectory(
                new SimplePathBuilder(new Vector2(-95, 218), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(-45, 231), Rotation2.fromDegrees(0))
                        .build(),
                bridgeConstraints, SAMPLE_DISTANCE);
        }        
    
    @SuppressWarnings("unused")
    private Path getPath(String name) throws IOException {
        InputStream in = getClass().getClassLoader().getResourceAsStream(name);
        if (in == null) {
            throw new FileNotFoundException("Path file not found: " + name);
        }

        try (PathReader reader = new PathReader(new InputStreamReader(in))) {
            return reader.read();
        }
    }

//#region getters
    public Trajectory getSevenFeet(){
        return sevenFeet;
    }

    public Trajectory get_sCurve(){
            return sCurve;
    }

    public Trajectory getThreeObjectFarPart1(boolean isBlue){return isBlue?threeObjectFarPart1Blue:threeObjectFarPart1;}

    public Trajectory getThreeObjectFarPart2(boolean isBlue){return isBlue?threeObjectFarPart2Blue:threeObjectFarPart2;}

    public Trajectory getThreeObjectFarPart3(boolean isBlue){return isBlue?threeObjectFarPart3Blue:threeObjectFarPart3;}

    public Trajectory getThreeObjectFarPart4(boolean isBlue){return isBlue?threeObjectFarPart4Blue:threeObjectFarPart4;}

    public Trajectory getThreeObjectFarPart5(boolean isBlue){return isBlue?threeObjectFarPart5Blue:threeObjectFarPart5;}

    public Trajectory getThreeObjectClosePart1(boolean isBlue){return isBlue?threeObjectClosePart1Blue:threeObjectClosePart1;}

    public Trajectory getThreeObjectClosePart2(boolean isBlue){return isBlue?threeObjectClosePart2Blue:threeObjectClosePart2;}

    public Trajectory getThreeObjectClosePart3(boolean isBlue){return isBlue?threeObjectClosePart3Blue:threeObjectClosePart3;}

    public Trajectory getThreeObjectClosePart4(boolean isBlue){return isBlue?threeObjectClosePart4Blue:threeObjectClosePart4;}

    public Trajectory getThreeObjectBridgePart1(boolean isBlue){return isBlue?threeObjectBridgePart1Blue:threeObjectBridgePart1;}

    public Trajectory getThreeObjectBridgePart2(boolean isBlue){return isBlue?threeObjectBridgePart2Blue:threeObjectBridgePart2;}

    public Trajectory getThreeObjectBridgePart3(boolean isBlue){return isBlue?threeObjectBridgePart3Blue:threeObjectBridgePart3;}

    public Trajectory getThreeObjectBridgePart4(boolean isBlue){return isBlue?threeObjectBridgePart4Blue:threeObjectBridgePart4;}

    public Trajectory getConeBridgeToPlace1(boolean isBlue){
        return isBlue?coneBridgeToPlace1Blue:coneBridgeToPlace1;
    }

    public Trajectory getConeBridgeToPlace2(boolean isBlue){
        return isBlue?coneBridgeToPlace2Blue:coneBridgeToPlace2;
    }

    public Trajectory getConeBridgeToPickup1(boolean isBlue){
        return isBlue?coneBridgeToPickUp1Blue:coneBridgeToPickUp1;
    }

    public Trajectory getConeBridgeToPickup2(boolean isBlue){return isBlue?coneBridgeToPickUp2Blue:coneBridgeToPickUp2;}

    public Trajectory getThreeObjectCloseEnd1(boolean isBlue){return isBlue?threeObjectCloseEnd1Blue:threeObjectCloseEnd1;}

    public Trajectory getThreeObjectCloseEnd2(boolean isBlue){return isBlue?threeObjectCloseEnd2Blue:threeObjectCloseEnd2;}

    public Trajectory getOnToBridge(){return onToBridge;}

    public Trajectory getPastBridge(){return goPastBridge;}

    public Trajectory getUpBridge(Vector2 start, Rotation2 rotation, int movement){
        return new Trajectory(
                new SimplePathBuilder(new Vector2(start.x, start.y), Rotation2.fromDegrees(rotation.toDegrees()))
                        .lineTo(new Vector2(start.x+movement, start.y), Rotation2.fromDegrees(rotation.toDegrees()))
                        .build(),
                bridgeConstraints, SAMPLE_DISTANCE);
    }

    public Trajectory getToBridge1(boolean isBlue){
        return isBlue?coneBridgeToBridge1Blue:coneBridgeToBridge1;
    }

    public Trajectory getToBridge2(){
        return coneBridgeToBridge2;
    }

    public Trajectory getToBridge2Half(){
        return coneBridgeToBridge2Half;
    }

    public Trajectory placeAndLeave(boolean isBlue){
        return new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(87, 0), Rotation2.fromDegrees(0))
                        .build(),
                        slowConstraints, SAMPLE_DISTANCE
        );
    }
//#endregion

public Trajectory getFromOverBridgeToCone() {
        return getToConePastBridge;
}
}
