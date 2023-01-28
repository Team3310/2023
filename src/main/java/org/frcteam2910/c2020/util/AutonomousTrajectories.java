package org.frcteam2910.c2020.util;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.io.PathReader;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Arrays;

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

    private Trajectory threeObjectBridgePart1;
    private Trajectory threeObjectBridgePart2;
    private Trajectory threeObjectBridgePart3;
    private Trajectory threeObjectBridgePart4;

    private Trajectory coneBridgefromPlaceToLoad;
    private Trajectory coneBridgefromLoadtoPlace;
    private Trajectory coneBridgefromLoadtoPickUp1;
    private Trajectory coneBridgefromLoadtoPickUp2;
    private Trajectory coneBridgefromPickUp1ToLoad;
    private Trajectory coneBridgefromPickUp2ToLoad;
//#endregion

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints, Boolean isBlue)throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);
        
        TrajectoryConstraint[] mediumConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        mediumConstraints[mediumConstraints.length - 1] = new MaxVelocityConstraint(9.0 * 12.0);
        mediumConstraints[mediumConstraints.length - 2] = new MaxAccelerationConstraint(8.0 * 12.0);

        if(isBlue){
            xReflect=-1;  
            angleOffset=0;      
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
//#endregion
//#region Three Object Close
        threeObjectClosePart1 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*259.5, 297.5), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*67.5, 278.5))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart2 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*67.5, 278.5), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*259.5, 297.5))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart3 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*259.5, 297.5), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*140.5, 286.5))
                        .lineTo(new Vector2(xReflect*67.5, 230))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart4 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*67.5, 230), Rotation2.fromDegrees(0+angleOffset))
                        .arcTo(new Vector2(xReflect*122.5, 273), new Vector2(xReflect*122.5, 217.6))
                        .lineTo(new Vector2(xReflect*253.5, 257))
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
//#endregion
//#region Cone Bridge
        coneBridgefromPlaceToLoad = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*251.5, 126), Rotation2.fromDegrees(0.0+angleOffset))
                        .lineTo(new Vector2(xReflect*130.5, 126))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        coneBridgefromLoadtoPlace = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*130.5, 126), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*251.5, 126))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        
        coneBridgefromLoadtoPickUp2 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*130.5, 126), Rotation2.fromDegrees(0+angleOffset))
                        .arcTo(new Vector2(xReflect*58.5, 186), new Vector2(xReflect*130.5, 189))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        coneBridgefromPickUp2ToLoad = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*58.5, 186), Rotation2.fromDegrees(0+angleOffset))
                        .arcTo(new Vector2(xReflect*130.5, 126), new Vector2(xReflect*130.5, 185))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        coneBridgefromLoadtoPickUp1 = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*130.5, 126), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*64.5, 135)/*, new Vector2(196, 237.5)*/)
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        coneBridgefromPickUp1ToLoad = new Trajectory(
                new SimplePathBuilder(new Vector2(xReflect*64.5, 135), Rotation2.fromDegrees(0+angleOffset))
                        .lineTo(new Vector2(xReflect*130.5, 126)/*, new Vector2(196, 180)*/)
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
//#endregion        
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

    public Trajectory getThreeObjectFarPart1(){return ThreeObjectFarPart1;}

    public Trajectory getThreeObjectFarPart2(){return ThreeObjectFarPart2;}

    public Trajectory getThreeObjectFarPart3(){return ThreeObjectFarPart3;}

    public Trajectory getThreeObjectFarPart4(){return ThreeObjectFarPart4;}

    public Trajectory getThreeObjectFarPart5(){return ThreeObjectFarPart5;}

    public Trajectory getThreeObjectClosePart1(){return threeObjectClosePart1;}

    public Trajectory getThreeObjectClosePart2(){return threeObjectClosePart2;}

    public Trajectory getThreeObjectClosePart3(){return threeObjectClosePart3;}

    public Trajectory getThreeObjectClosePart4(){return threeObjectClosePart4;}

    public Trajectory getThreeObjectBridgePart1(){return threeObjectBridgePart1;}

    public Trajectory getThreeObjectBridgePart2(){return threeObjectBridgePart2;}

    public Trajectory getThreeObjectBridgePart3(){return threeObjectBridgePart3;}

    public Trajectory getThreeObjectBridgePart4(){return threeObjectBridgePart4;}

    public Trajectory getConeBridgefromPickUp1ToLoad(){return coneBridgefromPickUp1ToLoad;}

    public Trajectory getConeBridgefromPickUp2ToLoad(){return coneBridgefromPickUp2ToLoad;}

    public Trajectory getConeBridgefromLoadtoPlace(){return coneBridgefromLoadtoPlace;}

    public Trajectory getConeBridgefromPlaceToLoad(){return coneBridgefromPlaceToLoad;}

    public Trajectory getConeBridgePickup1(){return coneBridgefromLoadtoPickUp1;}

    public Trajectory getConeBridgePickup2(){return coneBridgefromLoadtoPickUp2;}
//#endregion
}
