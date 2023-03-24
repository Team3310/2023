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

import edu.wpi.first.math.geometry.Rotation2d;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;
    private int xReflect;
    private double angleOffset;

//#region Trajectories
    private Trajectory sevenFeet;
    private final Trajectory sCurve;

    private Trajectory ThreeObjectFarPart1;
    private Trajectory ThreeObjectFarPart2;
    private Trajectory ThreeObjectFarPart3;
    private Trajectory ThreeObjectFarPart4;
    private Trajectory ThreeObjectFarPart5;

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
    private Trajectory coneBridgePickUp2;
    private Trajectory coneBridgeToPlace1;
    private Trajectory coneBridgeToPlace2;

    private Trajectory onToBridge;
    private Trajectory goPastBridge;
    private Trajectory getToConePastBridge;

    private Trajectory ThreeObjectFarPart1Blue;
    private Trajectory ThreeObjectFarPart2Blue;
    private Trajectory ThreeObjectFarPart3Blue;
    private Trajectory ThreeObjectFarPart4Blue;
    private Trajectory ThreeObjectFarPart5Blue;

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
        mediumConstraints[mediumConstraints.length - 1] = new MaxVelocityConstraint(5.0 * 12.0); //9
        mediumConstraints[mediumConstraints.length - 2] = new MaxAccelerationConstraint(3.0 * 12.0);//4

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
        ThreeObjectFarPart1 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*156.5,298), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*254.5, 298))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        ThreeObjectFarPart2 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*255.5,298), Rotation2.fromDegrees(0+angleOffset))//71,298 non spit
                        .lineTo(new Vector2(xReflect*67.5, 276))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        ThreeObjectFarPart3 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*67.5, 278), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*254.5, 268))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        ThreeObjectFarPart4 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*254.5, 268), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*122.5, 273))
                        //.lineTo(new Vector2(259, 228))
                        .arcTo(new Vector2(xReflect*67.5, 228), new Vector2(xReflect*122.5, 217.5))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        ThreeObjectFarPart5 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*67.5, 228), Rotation2.fromDegrees(0+angleOffset))
                        .arcTo(new Vector2(xReflect*122.5, 273), new Vector2(xReflect*122.5, 217.6))
                        .lineTo(new Vector2(xReflect*220.5, 273))
                        .arcTo(new Vector2(xReflect*252.5, 228), new Vector2(xReflect*121, 230))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        ThreeObjectFarPart1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*156.5,298), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*254.5, 298))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        ThreeObjectFarPart2Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*255.5,298), Rotation2.fromDegrees(0+angleOffset))//71,298 non spit
                        .lineTo(new Vector2(xReflect*67.5, 276))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        ThreeObjectFarPart3Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*67.5, 278), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*254.5, 268))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        ThreeObjectFarPart4Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*254.5, 268), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*122.5, 273))
                        //.lineTo(new Vector2(259, 228))
                        .arcTo(new Vector2(xReflect*67.5, 228), new Vector2(xReflect*122.5, 217.5))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        ThreeObjectFarPart5Blue = new Trajectory(
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
        coneBridgePickUp2 = new Trajectory(
                new SimplePathBuilder(new Vector2(-250, 140.68), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(-123.3, 140.68), Rotation2.fromDegrees(30))
                        .arcTo(new Vector2(-58.1, 182.565), new Vector2(-123.3, 228.22))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        coneBridgeToPlace2 = new Trajectory(
                new SimplePathBuilder(new Vector2(-58.1, 182.565), Rotation2.fromDegrees(30))
                        .arcTo(new Vector2(-123.3, 140.68), new Vector2(-123.3, 228.22))
                        .lineTo(new Vector2(-250, 146.68), Rotation2.fromDegrees(180))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        coneBridgeToPickUp1 = new Trajectory(
                new SimplePathBuilder(new Vector2(-250, 119), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(-58.1, 130.5))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        coneBridgeToPlace1 = new Trajectory(
                new SimplePathBuilder(new Vector2(-58.1, 130.5), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(-250, 140.68))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );

        //#region blue
                coneBridgeToPickUp2Blue = new Trajectory(
                        new SimplePathBuilder(new Vector2(-250, 140.68), Rotation2.fromDegrees(180))
                                .lineTo(new Vector2(-123.3, 140.68))
                                .arcTo(new Vector2(-46.96, 182.565), new Vector2(-123.3, 228.22))
                                .build(),
                        mediumConstraints, SAMPLE_DISTANCE
                );

                coneBridgeToPlace2Blue = new Trajectory(
                        new SimplePathBuilder(new Vector2(-46.96, 182.565), Rotation2.fromDegrees(180))
                                .arcTo(new Vector2(-123.3, 140.68), new Vector2(-123.3, 228.22))
                                .lineTo(new Vector2(-250, 140.68))
                                .build(),
                        mediumConstraints, SAMPLE_DISTANCE
                );

                coneBridgeToPickUp1Blue = new Trajectory(
                        new SimplePathBuilder(new Vector2(250, 119), Rotation2.fromDegrees(180))
                                .lineTo(new Vector2(58.1, 130.5))
                                .build(),
                        mediumConstraints, SAMPLE_DISTANCE
                );

                coneBridgeToPlace1Blue = new Trajectory(
                        new SimplePathBuilder(new Vector2(58.1, 130.5), Rotation2.fromDegrees(180))
                                .lineTo(new Vector2(256, 140.68))
                                .build(),
                        mediumConstraints, SAMPLE_DISTANCE
                );
        //#endregion
//#endregion        
        onToBridge = new Trajectory(
                new SimplePathBuilder(new Vector2(-252, 207), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(-173, 207), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(-176, 207), Rotation2.fromDegrees(0))
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

    public Trajectory getThreeObjectFarPart1(boolean isBlue){return isBlue?ThreeObjectFarPart1Blue:ThreeObjectFarPart1;}

    public Trajectory getThreeObjectFarPart2(boolean isBlue){return isBlue?ThreeObjectFarPart2Blue:ThreeObjectFarPart2;}

    public Trajectory getThreeObjectFarPart3(boolean isBlue){return isBlue?ThreeObjectFarPart3Blue:ThreeObjectFarPart3;}

    public Trajectory getThreeObjectFarPart4(boolean isBlue){return isBlue?ThreeObjectFarPart4Blue:ThreeObjectFarPart4;}

    public Trajectory getThreeObjectFarPart5(boolean isBlue){return isBlue?ThreeObjectFarPart5Blue:ThreeObjectFarPart5;}

    public Trajectory getThreeObjectClosePart1(boolean isBlue){return isBlue?threeObjectClosePart1Blue:threeObjectClosePart1;}

    public Trajectory getThreeObjectClosePart2(boolean isBlue){return isBlue?threeObjectClosePart2Blue:threeObjectClosePart2;}

    public Trajectory getThreeObjectClosePart3(boolean isBlue){return isBlue?threeObjectClosePart3Blue:threeObjectClosePart3;}

    public Trajectory getThreeObjectClosePart4(boolean isBlue){return isBlue?threeObjectClosePart4Blue:threeObjectClosePart4;}

    public Trajectory getThreeObjectBridgePart1(boolean isBlue){return isBlue?threeObjectBridgePart1Blue:threeObjectBridgePart1;}

    public Trajectory getThreeObjectBridgePart2(boolean isBlue){return isBlue?threeObjectBridgePart2Blue:threeObjectBridgePart2;}

    public Trajectory getThreeObjectBridgePart3(boolean isBlue){return isBlue?threeObjectBridgePart3Blue:threeObjectBridgePart3;}

    public Trajectory getThreeObjectBridgePart4(boolean isBlue){return isBlue?threeObjectBridgePart4Blue:threeObjectBridgePart4;}

    public Trajectory getConeBridgeToPlace1(boolean isBlue){return isBlue?coneBridgeToPlace1Blue:coneBridgeToPlace1;}

    public Trajectory getConeBridgeToPlace2(boolean isBlue){return isBlue?coneBridgeToPlace2Blue:coneBridgeToPlace2;}

    public Trajectory getConeBridgeToPickup1(boolean isBlue){return isBlue?coneBridgeToPickUp1Blue:coneBridgeToPickUp1;}

    public Trajectory getConeBridgeToPickup2(boolean isBlue){return isBlue?coneBridgeToPickUp2Blue:coneBridgePickUp2;}

    public Trajectory geTthreeObjectCloseEnd1(boolean isBlue){return isBlue?threeObjectCloseEnd1Blue:threeObjectCloseEnd1;}

    public Trajectory geTthreeObjectCloseEnd2(boolean isBlue){return isBlue?threeObjectCloseEnd2Blue:threeObjectCloseEnd2;}

    public Trajectory getOnToBridge(){return onToBridge;}

    public Trajectory getPastBridge(){return goPastBridge;}

    public Trajectory getUpBridge(Vector2 start, Rotation2 rotation, int movement){
        return new Trajectory(
                new SimplePathBuilder(new Vector2(start.x, start.y), Rotation2.fromDegrees(rotation.toDegrees()))
                        .lineTo(new Vector2(start.x+movement, start.y), Rotation2.fromDegrees(rotation.toDegrees()))
                        .build(),
                bridgeConstraints, SAMPLE_DISTANCE);
    }

    public Trajectory getToBridge(Vector2 start, Rotation2 rotation){
        return new Trajectory(
                new SimplePathBuilder(start, Rotation2.fromDegrees(rotation.toDegrees()))
                        .lineTo(new Vector2(-252, 207), Rotation2.fromDegrees(180))
                        .build(),
                bridgeConstraints, SAMPLE_DISTANCE);
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
