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

public class AutonomousTrajectories
{
    private static final double SAMPLE_DISTANCE = 0.1;
    private TrajectoryConstraint[] bridgeConstraints;
    private TrajectoryConstraint[] mediumConstraints;
    private TrajectoryConstraint[] mediumFastConstraints;
    private TrajectoryConstraint[] mediumSlowConstraints;
    private TrajectoryConstraint[] slowConstraints;
    private TrajectoryConstraint[] bumpConstraints;

//#region Trajectories
    private final Trajectory sevenFeet;
    private final Trajectory sCurve;

    private final Trajectory threeObjectFarPart1;
    private final Trajectory threeObjectFarPart2;
    private final Trajectory threeObjectFarPart3;
    private final Trajectory threeObjectFarPart4A;
    private final Trajectory threeObjectFarPart4B;

    private final Trajectory threeObjectClosePart1;
    private final Trajectory threeObjectClosePart2;
    private final Trajectory threeObjectClosePart3;
    private final Trajectory threeObjectClosePart4;
    private final Trajectory threeObjectCloseEnd1;
    private final Trajectory threeObjectCloseEnd2;

    private final Trajectory threeObjectBridgePart1;
    private final Trajectory threeObjectBridgePart2;
    private final Trajectory threeObjectBridgePart3;
    private final Trajectory threeObjectBridgePart4;

    private final Trajectory easySideConeToPickUp1;
    private final Trajectory easySideConeToPickUp2;
    private final Trajectory easySideConeToPlace1;
    private final Trajectory easySideConeToPlace2;
    private final Trajectory easySideToEndSpot;
    private final Trajectory easySideToEndSpotBlue;

    private final Trajectory onToBridge;
    private final Trajectory goPastBridge;
    private final Trajectory getToConePastBridge;

    private final Trajectory threeObjectFarPart1Blue;
    private final Trajectory threeObjectFarPart2Blue;
    private final Trajectory threeObjectFarPart3Blue;
    private final Trajectory threeObjectFarPart4ABlue;
    private final Trajectory threeObjectFarPart4BBlue;
    private final Trajectory threeObjectFarPart5Blue;
    private final Trajectory threeObjectFarPart5;

    private final Trajectory threeObjectClosePart1Blue;
    private final Trajectory threeObjectClosePart2Blue;
    private final Trajectory threeObjectClosePart3Blue;
    private final Trajectory threeObjectClosePart4Blue;
    private final Trajectory threeObjectCloseEnd1Blue;
    private final Trajectory threeObjectCloseEnd2Blue;

    private final Trajectory threeObjectBridgePart1Blue;
    private final Trajectory threeObjectBridgePart2Blue;
    private final Trajectory threeObjectBridgePart3Blue;
    private final Trajectory threeObjectBridgePart4Blue;

    private final Trajectory easySideConeToPickUp1Blue;
    private final Trajectory easySideConeToPickUp2Blue;
    private final Trajectory easySideConeToPlace1Blue;
    private final Trajectory easySideConeToPlace2Blue;
//     private final Trajectory easySideConeToBridge1;
    private final Trajectory easySideConeToBridge1Blue;
    private final Trajectory easySideConeToBridge2;
    private final Trajectory easySideConeToBridge2Half;
//#endregion

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints, SideMode side){

        System.out.println("ran trajectories constructor");
        //#region constraints
        slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);
        
        mediumConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        mediumConstraints[mediumConstraints.length - 1] = new MaxVelocityConstraint(18.0 * 12.0); //9
        mediumConstraints[mediumConstraints.length - 2] = new MaxAccelerationConstraint(6 * 12.0);//4

        mediumFastConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        mediumFastConstraints[mediumConstraints.length - 1] = new MaxVelocityConstraint(18.0 * 12.0); //9
        mediumFastConstraints[mediumConstraints.length - 2] = new MaxAccelerationConstraint(7 * 12.0);//4

        mediumSlowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        mediumSlowConstraints[mediumConstraints.length - 1] = new MaxVelocityConstraint(10.0 * 12.0); //9
        mediumSlowConstraints[mediumConstraints.length - 2] = new MaxAccelerationConstraint(7 * 12.0);//4


        bridgeConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        bridgeConstraints[mediumConstraints.length - 1] = new MaxVelocityConstraint(2.0 * 12.0);
        bridgeConstraints[mediumConstraints.length - 2] = new MaxAccelerationConstraint(2.0 * 12.0);

        bumpConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        bumpConstraints[mediumConstraints.length - 1] = new MaxVelocityConstraint(2.0 * 12.0);
        bumpConstraints[mediumConstraints.length - 2] = new MaxAccelerationConstraint(1.0 * 12.0);
        //#endregion

        sCurve = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(60, 0), Rotation2.ZERO)
                        .arcTo(new Vector2(100, 40), new Vector2(60, 40))
                        .lineTo(new Vector2(100, 60), Rotation2.ZERO)
                        .arcTo(new Vector2(140, 100), new Vector2(140, 60), Rotation2.ZERO)
                        .lineTo(new Vector2(200, 100))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

//#region Three Object Bump
        //#region Red - Three Object Far
        threeObjectFarPart1 = new Trajectory(
                new SimplePathBuilder(new Vector2(0,0), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(160, 0))
                        .lineTo(new Vector2(223, -24))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart2 = new Trajectory(
                new SimplePathBuilder(getEndCoords(threeObjectFarPart1), getEndRotation(threeObjectFarPart1))
                        .lineTo(new Vector2(188, -15), Rotation2.fromDegrees(-5))
                        .lineTo(new Vector2(148, -15))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart3 = new Trajectory(
                new SimplePathBuilder(getEndCoords(threeObjectFarPart2), getEndRotation(threeObjectFarPart2))
                        .lineTo(new Vector2(168, -15), Rotation2.fromDegrees(155))
                        .lineTo(new Vector2(219, -72))
                        .lineTo(new Vector2(237, -116))
                        .build(),
                mediumSlowConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart4A = new Trajectory(
                new SimplePathBuilder(getEndCoords(threeObjectFarPart3), getEndRotation(threeObjectFarPart3))
                        .lineTo(new Vector2(178, -20), Rotation2.fromDegrees(10))
                        .lineTo(new Vector2(148, -20))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart4B = new Trajectory(
                new SimplePathBuilder(getEndCoords(threeObjectFarPart3), getEndRotation(threeObjectFarPart3))
                        .lineTo(new Vector2(173, -15), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(68, -15))
                        .lineTo(new Vector2(-3, -40), Rotation2.fromDegrees(195))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart5 = new Trajectory(
                new SimplePathBuilder(getEndCoords(threeObjectFarPart4A), getEndRotation(threeObjectFarPart4A))
                        .lineTo(new Vector2(190, -20), Rotation2.fromDegrees(-180))
                        .lineTo(new Vector2(190, -138), Rotation2.fromDegrees(-180))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        //#endregion
        //#region Blue - Three Object Far
        threeObjectFarPart1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(0,0), Rotation2.fromDegrees(-180))
                        .lineTo(new Vector2(160, 0))
                        .lineTo(new Vector2(223, 18))
                        .build(),
                mediumFastConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart2Blue = new Trajectory(
                new SimplePathBuilder(getEndCoords(threeObjectFarPart1Blue), getEndRotation(threeObjectFarPart1Blue))
                        .lineTo(new Vector2(188, 15), Rotation2.fromDegrees(5))
                        .lineTo(new Vector2(148, 15))
                        .build(),
                mediumFastConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart3Blue = new Trajectory(
                new SimplePathBuilder(getEndCoords(threeObjectFarPart2Blue), getEndRotation(threeObjectFarPart2Blue))
                        .lineTo(new Vector2(168, 15), Rotation2.fromDegrees(-170))
                        // .lineTo(new Vector2(219, 82))
                        .lineTo(new Vector2(237, 82))
                        .build(),
                mediumFastConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart4ABlue = new Trajectory(
                new SimplePathBuilder(getEndCoords(threeObjectFarPart3Blue), getEndRotation(threeObjectFarPart3Blue))
                        .lineTo(new Vector2(178, 18), Rotation2.fromDegrees(-10))
                        .lineTo(new Vector2(148, 18))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart4BBlue = new Trajectory(
                new SimplePathBuilder(getEndCoords(threeObjectFarPart3Blue), getEndRotation(threeObjectFarPart3Blue))
                        .lineTo(new Vector2(173, 15), Rotation2.fromDegrees(-180))
                        .lineTo(new Vector2(68, 5))
                        .lineTo(new Vector2(-51, 40), Rotation2.fromDegrees(-180))
                        .build(),
                mediumFastConstraints, SAMPLE_DISTANCE
        );
        threeObjectFarPart5Blue = new Trajectory(
                new SimplePathBuilder(getEndCoords(threeObjectFarPart4ABlue), getEndRotation(threeObjectFarPart4ABlue))
                        .lineTo(new Vector2(200, 18), Rotation2.fromDegrees(-180))
                        .lineTo(new Vector2(200, 158), Rotation2.fromDegrees(-180))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        //#endregion
//#endregion
//#region Three Object IDK
        threeObjectClosePart1 = new Trajectory(
                new SimplePathBuilder(new Vector2(259.5, 297.5), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(58.75, 279))
                        .build(),
                bumpConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart2 = new Trajectory(
                new SimplePathBuilder(new Vector2(68.75, 279), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(259.5, 297.5))
                        .build(),
                bumpConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart3 = new Trajectory(
                new SimplePathBuilder(new Vector2(259.5, 297.5), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(135, 286))
                        .lineTo(new Vector2(58.5, 230.5))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart4 = new Trajectory(
                new SimplePathBuilder(new Vector2(58.5, 230.5), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(135, 286))
                        .lineTo(new Vector2(227.6, 279))
                        .lineTo(new Vector2(258, 250))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectCloseEnd1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(259.5, 297.5), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(237.9, 230))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        threeObjectCloseEnd2Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(258, 260), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(237.9, 230))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(259.5, 297.5), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(58.75, 279))
                        .build(),
                bumpConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart2Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(68.75, 279), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(259.5, 297.5))
                        .build(),
                bumpConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart3Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(259.5, 297.5), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(135, 286))
                        .lineTo(new Vector2(58.5, 230.5))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart4Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(58.5, 230.5), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(135, 286))
                        .lineTo(new Vector2(227.6, 279))
                        .lineTo(new Vector2(258, 250))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
//#endregion
//#region Three Object Bridge
        threeObjectBridgePart1 = new Trajectory(
                new SimplePathBuilder(new Vector2(254.5, 161.5), Rotation2.fromDegrees(0))
                        .arcTo(new Vector2(238.5, 128), new Vector2(235.5, 158))
                        .lineTo(new Vector2(66, 128))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart2 = new Trajectory(
                new SimplePathBuilder(new Vector2(66, 128), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(251.5, 138))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart3 = new Trajectory(
                new SimplePathBuilder(new Vector2(251.5, 138), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(126.5, 130))
                        .arcTo(new Vector2(56.5, 182), new Vector2(126.5, 201))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart4 = new Trajectory(
                new SimplePathBuilder(new Vector2(56.5, 182), Rotation2.fromDegrees(0))
                        .arcTo(new Vector2(129.5, 131), new Vector2(135.5, 201.5))
                        .lineTo(new Vector2(251.5, 126))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(254.5, 161.5), Rotation2.fromDegrees(0))
                        .arcTo(new Vector2(238.5, 128), new Vector2(235.5, 158))
                        .lineTo(new Vector2(66, 128))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart2Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(66, 128), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(251.5, 138))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart3Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(251.5, 138), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(126.5, 130))
                        .arcTo(new Vector2(56.5, 182), new Vector2(126.5, 201))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart4Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(56.5, 182), Rotation2.fromDegrees(0))
                        .arcTo(new Vector2(129.5, 131), new Vector2(135.5, 201.5))
                        .lineTo(new Vector2(251.5, 126))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
//#endregion
//#region Easy Side
        //#region Red - Easy Side
        easySideConeToPickUp1 =
                new Trajectory(new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(148, 0))
                        .lineTo(new Vector2(217, 24.0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        easySideConeToPlace1 = new Trajectory(
                new SimplePathBuilder(getEndCoords(easySideConeToPickUp1), getEndRotation(easySideConeToPickUp1))
                        .lineTo(new Vector2(211, 12))
                        .lineTo(new Vector2(48, 12))
                        .lineTo(new Vector2(-5.0, 24.0))
                        .build(),
                mediumFastConstraints, SAMPLE_DISTANCE);


        easySideConeToPickUp2 = new Trajectory(
                new SimplePathBuilder(getEndCoords(easySideConeToPlace1), getEndRotation(easySideConeToPlace1))
                        .lineTo(new Vector2(48, 12))
                        .lineTo(new Vector2(108, 14))
                        .lineTo(new Vector2(138, 14))
                        .lineTo(new Vector2(217, 96), Rotation2.fromDegrees(240))
                        .build(),
                mediumFastConstraints, SAMPLE_DISTANCE);        

        easySideConeToPlace2 = new Trajectory(
                new SimplePathBuilder(getEndCoords(easySideConeToPickUp2), getEndRotation(easySideConeToPickUp2))
                        .lineTo(new Vector2(178, 42), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(50, 42))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE); 
                 
        easySideToEndSpot = new Trajectory(
                new SimplePathBuilder(getEndCoords(easySideConeToPlace2), getEndRotation(easySideConeToPlace2))
                        .lineTo(new Vector2(190, 18), Rotation2.fromDegrees(180))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE); 
        //#endregion
        //#region Blue - Easy Side
        easySideConeToPickUp1Blue =
                new Trajectory(new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(148, -6))
                        .lineTo(new Vector2(217, -24.0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        easySideConeToPlace1Blue = new Trajectory(
                new SimplePathBuilder(getEndCoords(easySideConeToPickUp1Blue), getEndRotation(easySideConeToPickUp1Blue))
                        .lineTo(new Vector2(211, -18))
                        .lineTo(new Vector2(48, -18))
                        .lineTo(new Vector2(-5.0, -24.0))
                        .build(),
                mediumFastConstraints, SAMPLE_DISTANCE);


        easySideConeToPickUp2Blue = new Trajectory(
                new SimplePathBuilder(getEndCoords(easySideConeToPlace1Blue), getEndRotation(easySideConeToPlace1Blue))
                        .lineTo(new Vector2(48, -16))
                        .lineTo(new Vector2(108, -16))
                        .lineTo(new Vector2(138, -18))
                        .lineTo(new Vector2(217, -96), Rotation2.fromDegrees(-240))
                        .build(),
                mediumFastConstraints, SAMPLE_DISTANCE);        

        easySideConeToPlace2Blue = new Trajectory(
                new SimplePathBuilder(getEndCoords(easySideConeToPickUp2Blue), getEndRotation(easySideConeToPickUp2Blue))
                        .lineTo(new Vector2(178, -42), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(50, -42))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        easySideToEndSpotBlue = new Trajectory(
                new SimplePathBuilder(getEndCoords(easySideConeToPlace2Blue), getEndRotation(easySideConeToPlace2Blue))
                        .lineTo(new Vector2(192, -18), Rotation2.fromDegrees(180))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE); 
                        
        easySideConeToBridge1Blue = new Trajectory(
                new SimplePathBuilder(new Vector2(-246, 140.68), Rotation2.fromDegrees(200))
                        .lineTo(new Vector2(-240, 140.68))
                        .lineTo(new Vector2(-240, 197.68))
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

        //TODO fill out path or remove
        easySideConeToBridge2 = new Trajectory(
                new SimplePathBuilder(new Vector2(-95, 218), Rotation2.fromDegrees(0))
                .lineTo(new Vector2(-45, 231), Rotation2.fromDegrees(0))
                .build(),
        bridgeConstraints, SAMPLE_DISTANCE);

        easySideConeToBridge2Half = new Trajectory(
                new SimplePathBuilder(new Vector2(-95, 218), Rotation2.fromDegrees(0))
                .lineTo(new Vector2(-45, 231), Rotation2.fromDegrees(0))
                .build(),
        bridgeConstraints, SAMPLE_DISTANCE);

        threeObjectCloseEnd1 = new Trajectory(
                new SimplePathBuilder(new Vector2(-95, 218), Rotation2.fromDegrees(0))
                .lineTo(new Vector2(-45, 231), Rotation2.fromDegrees(0))
                .build(),
        bridgeConstraints, SAMPLE_DISTANCE);

        threeObjectCloseEnd2 = new Trajectory(
                new SimplePathBuilder(new Vector2(-95, 218), Rotation2.fromDegrees(0))
                .lineTo(new Vector2(-45, 231), Rotation2.fromDegrees(0))
                .build(),
        bridgeConstraints, SAMPLE_DISTANCE);

        sevenFeet = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(180))
                .lineTo(new Vector2(11*12, 0))
                .build(),
        mediumSlowConstraints, SAMPLE_DISTANCE, 0.0 ,5.0);

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
    public Trajectory getFromOverBridgeToCone() {
        return getToConePastBridge;
    }

    public Trajectory getSevenFeet(){
        return sevenFeet;
    }

    public Trajectory get_sCurve(){
            return sCurve;
    }

    public Trajectory getThreeObjectFarPart1(boolean isBlue){return isBlue?threeObjectFarPart1Blue:threeObjectFarPart1;}

    public Trajectory getThreeObjectFarPart2(boolean isBlue){return isBlue?threeObjectFarPart2Blue:threeObjectFarPart2;}

    public Trajectory getThreeObjectFarPart3(boolean isBlue){return isBlue?threeObjectFarPart3Blue:threeObjectFarPart3;}

    public Trajectory getThreeObjectFarPart4A(boolean isBlue){return isBlue?threeObjectFarPart4ABlue:threeObjectFarPart4A;}

    public Trajectory getThreeObjectFarPart4B(boolean isBlue){return isBlue?threeObjectFarPart4BBlue:threeObjectFarPart4B;}

    public Trajectory getThreeObjectFarPart5(boolean isBlue){return isBlue?threeObjectFarPart5Blue:threeObjectFarPart5;}

    public Trajectory getThreeObjectClosePart1(boolean isBlue){return isBlue?threeObjectClosePart1Blue:threeObjectClosePart1;}

    public Trajectory getThreeObjectClosePart2(boolean isBlue){return isBlue?threeObjectClosePart2Blue:threeObjectClosePart2;}

    public Trajectory getThreeObjectClosePart3(boolean isBlue){return isBlue?threeObjectClosePart3Blue:threeObjectClosePart3;}

    public Trajectory getThreeObjectClosePart4(boolean isBlue){return isBlue?threeObjectClosePart4Blue:threeObjectClosePart4;}

    public Trajectory getThreeObjectBridgePart1(boolean isBlue){return isBlue?threeObjectBridgePart1Blue:threeObjectBridgePart1;}

    public Trajectory getThreeObjectBridgePart2(boolean isBlue){return isBlue?threeObjectBridgePart2Blue:threeObjectBridgePart2;}

    public Trajectory getThreeObjectBridgePart3(boolean isBlue){return isBlue?threeObjectBridgePart3Blue:threeObjectBridgePart3;}

    public Trajectory getThreeObjectBridgePart4(boolean isBlue){return isBlue?threeObjectBridgePart4Blue:threeObjectBridgePart4;}

    public Trajectory getBackwardsSevenFeet(){
        return new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(180))
                .lineTo(new Vector2(-11*12, 0))
                .build(),
        mediumSlowConstraints, SAMPLE_DISTANCE, 0.0 ,5.0);
    }

    public Trajectory getBackwardsXFeet(int x, double rotation){
        return new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(rotation))
                .lineTo(new Vector2(-x*12, 0))
                .build(),
        mediumSlowConstraints, SAMPLE_DISTANCE, 0.0 ,5.0);
    }

    public Trajectory getEasySideConeToPlace1(boolean isBlue){
        return isBlue?easySideConeToPlace1Blue:easySideConeToPlace1;
    }

    public Trajectory getEasySideConeToPlace2(boolean isBlue){
        return isBlue?easySideConeToPlace2Blue:easySideConeToPlace2;
    }

    public Trajectory getEasySideToEndSpot(boolean isBlue){
        return isBlue?easySideToEndSpotBlue:easySideToEndSpot;
    }

    public Trajectory getEasySideConeToPickup1(boolean isBlue){
        return isBlue?easySideConeToPickUp1Blue:easySideConeToPickUp1;
    }

    public Trajectory getEasySideConeToPickup2(boolean isBlue){
        return isBlue?easySideConeToPickUp2Blue:easySideConeToPickUp2;
    }

    @Deprecated
    public Trajectory getEasySideToBridge1(boolean isBlue){
        return isBlue?easySideConeToBridge1Blue:null;//easySideConeToBridge1;
    }

    public Trajectory getEasySideToBridge2(){
        return easySideConeToBridge2;
    }

    public Trajectory getEasySideToBridge2Half(){
        return easySideConeToBridge2Half;
    }

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


    public Trajectory placeAndLeave(boolean isBlue){
        return new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(87, 0), Rotation2.fromDegrees(0))
                        .build(),
                        slowConstraints, SAMPLE_DISTANCE
        );
    }
//#endregion

    public Vector2 getEndCoords(Trajectory traj){
        return traj.calculate(traj.getDuration()).getPathState().getPosition();
    }

    public Rotation2 getEndRotation(Trajectory traj){
        return traj.calculate(traj.getDuration()).getPathState().getRotation();
   }
}
