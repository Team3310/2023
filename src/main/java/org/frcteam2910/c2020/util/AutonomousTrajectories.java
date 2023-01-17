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

    private Trajectory testPart1;
    private Trajectory testPart2;
    private Trajectory testPart3;
    private Trajectory testPart4;
    private Trajectory testPart5;

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

        sevenFeet = new Trajectory(
                new SimplePathBuilder(new Vector2(0,0), Rotation2.ZERO)
                        .lineTo(new Vector2(84, 0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        testPart1 = new Trajectory(
                new SimplePathBuilder(new Vector2(176,278.5), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(78, 278.5), Rotation2.fromDegrees(180))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        testPart2 = new Trajectory(
                new SimplePathBuilder(new Vector2(78,278.5), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(275, 284), Rotation2.ZERO)
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        testPart3 = new Trajectory(
                new SimplePathBuilder(new Vector2(275, 284), Rotation2.ZERO)
                        .lineTo(new Vector2(78, 278.5), Rotation2.fromDegrees(180))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        testPart4 = new Trajectory(
                new SimplePathBuilder(new Vector2(78, 278.5), Rotation2.fromDegrees(180))
                        .lineTo(new Vector2(275, 236), Rotation2.ZERO)
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        testPart5 = new Trajectory(
                new SimplePathBuilder(new Vector2(275, 236), Rotation2.ZERO)
                        .lineTo(new Vector2(78, 300), Rotation2.fromDegrees(180))
                        .build(),
                mediumConstraints, SAMPLE_DISTANCE
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

    public Trajectory getTestPart1(){
        return testPart1;
    }

    public Trajectory getTestPart2(){
        return testPart2;
    }

    public Trajectory getTestPart3(){
        return testPart3;
    }

    public Trajectory getTestPart4(){
        return testPart4;
    }

    public Trajectory getTestPart5(){
        return testPart5;
    }
}
