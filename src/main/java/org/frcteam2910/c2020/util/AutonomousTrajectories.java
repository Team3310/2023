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

//     private static final String EIGHT_BALL_AUTO_PART_ONE_NAME = "autos/8BallAuto/8BallAutoPart1.path";
//     private static final String EIGHT_BALL_AUTO_PART_TWO_NAME = "autos/8BallAuto/8BallAutoPart2.path";

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

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);
        
        TrajectoryConstraint[] mediumConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        mediumConstraints[mediumConstraints.length - 1] = new MaxVelocityConstraint(9.0 * 12.0);
        mediumConstraints[mediumConstraints.length - 2] = new MaxAccelerationConstraint(8.0 * 12.0);

        sCurve = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(60, 0), Rotation2.ZERO)
                        .arcTo(new Vector2(100, 40), new Vector2(60, 40))
                        .lineTo(new Vector2(100, 60), Rotation2.ZERO)
                        .arcTo(new Vector2(140, 100), new Vector2(140, 60), Rotation2.ZERO)
                        .lineTo(new Vector2(200, 100))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE);

        ThreeObjectFarPart1 = new Trajectory(
                new SimplePathBuilder(new Vector2(170,298), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(71, 298), Rotation2.fromDegrees(0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        ThreeObjectFarPart2 = new Trajectory(
                new SimplePathBuilder(new Vector2(71,298), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(259, 276), Rotation2.fromDegrees(0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        ThreeObjectFarPart3 = new Trajectory(
                new SimplePathBuilder(new Vector2(257, 278), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(75, 268), Rotation2.fromDegrees(0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        ThreeObjectFarPart4 = new Trajectory(
                new SimplePathBuilder(new Vector2(75, 268), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(204, 273), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(259, 228))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        ThreeObjectFarPart5 = new Trajectory(
                new SimplePathBuilder(new Vector2(258, 230), Rotation2.fromDegrees(0))
                        .arcTo(new Vector2(204, 273), new Vector2(204, 217.6))
                        .lineTo(new Vector2(106, 273))
                        .arcTo(new Vector2(78, 228), new Vector2(121, 230))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart1 = new Trajectory(
                new SimplePathBuilder(new Vector2(67, 297.5), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(252, 278.5))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart2 = new Trajectory(
                new SimplePathBuilder(new Vector2(252, 278.5), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(67, 297.5))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart3 = new Trajectory(
                new SimplePathBuilder(new Vector2(67, 297.5), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(186, 286.5))
                        .lineTo(new Vector2(252, 230))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectClosePart4 = new Trajectory(
                new SimplePathBuilder(new Vector2(252, 230), Rotation2.fromDegrees(0))
                        .arcTo(new Vector2(204, 273), new Vector2(204, 217.6))
                        .lineTo(new Vector2(72, 257))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart1 = new Trajectory(
                new SimplePathBuilder(new Vector2(67, 161.5), Rotation2.fromDegrees(0))
                        .arcTo(new Vector2(88, 135), new Vector2(91, 158))
                        .lineTo(new Vector2(256.5, 135))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart2 = new Trajectory(
                new SimplePathBuilder(new Vector2(256.5, 135), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(67, 140))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart3 = new Trajectory(
                new SimplePathBuilder(new Vector2(67, 140), Rotation2.fromDegrees(0))
                        .lineTo(new Vector2(200, 140))
                        .arcTo(new Vector2(258, 182), new Vector2(200, 201))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeObjectBridgePart4 = new Trajectory(
                new SimplePathBuilder(new Vector2(258, 182), Rotation2.fromDegrees(0))
                        .arcTo(new Vector2(197, 131), new Vector2(190, 201.5))
                        .lineTo(new Vector2(67, 118.5))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
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

    public Trajectory getSevenFeet(){
        return sevenFeet;
    }

    public Trajectory get_sCurve(){
            return sCurve;
    }

    public Trajectory getThreeObjectFarPart1(){
        return ThreeObjectFarPart1;
    }

    public Trajectory getThreeObjectFarPart2(){
        return ThreeObjectFarPart2;
    }

    public Trajectory getThreeObjectFarPart3(){
        return ThreeObjectFarPart3;
    }

    public Trajectory getThreeObjectFarPart4(){
        return ThreeObjectFarPart4;
    }

    public Trajectory getThreeObjectFarPart5(){
        return ThreeObjectFarPart5;
    }

    public Trajectory getThreeObjectClosePart1(){
        return threeObjectClosePart1;
    }

    public Trajectory getThreeObjectClosePart2(){
        return threeObjectClosePart2;
    }

    public Trajectory getThreeObjectClosePart3(){
        return threeObjectClosePart3;
    }

    public Trajectory getThreeObjectClosePart4(){
        return threeObjectClosePart4;
    }

    public Trajectory getThreeObjectBridgePart1(){
        return threeObjectBridgePart1;
    }

    public Trajectory getThreeObjectBridgePart2(){
        return threeObjectBridgePart2;
    }

    public Trajectory getThreeObjectBridgePart3(){
        return threeObjectBridgePart3;
    }

    public Trajectory getThreeObjectBridgePart4(){
        return threeObjectBridgePart4;
    }
}
