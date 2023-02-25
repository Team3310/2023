package org.frcteam2910.c2020.util;

import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.util.BallColor;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriverReadout {
    private final SendableChooser<BallColor> ballColorChooser = new SendableChooser<>();

    public DriverReadout(RobotContainer container) {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver Readout");

        ballColorChooser.setDefaultOption("Red", BallColor.RED);
        ballColorChooser.addOption("Blue", BallColor.BLUE);

        tab.add("Autonomous Mode", container.getAutonomousChooser().getAutonomousModeChooser())
                .withSize(2, 1)
                .withPosition(2, 0);
        tab.add("Side", container.getSideChooser().getSendableChooser())
                .withSize(2, 1)
                .withPosition(2, 1);
        tab.add("Zero Gyroscope", new ZeroGyroscope(container.getDrivetrainSubsystem()))
                .withSize(2, 1)
                .withPosition(4, 0);
        tab.add("Gyro Auto Correct", container.getGyroAutoAdjustMode().getSendableChooser())
                .withSize(1, 1)
                .withPosition(4, 1);
    }

    public BallColor getTrackedBallColor() {
        return ballColorChooser.getSelected();
    }

    private static class ZeroGyroscope extends CommandBase {
        private final DrivetrainSubsystem drivetrain;

        public ZeroGyroscope(DrivetrainSubsystem drivetrain) {
            this.drivetrain = drivetrain;

            setName("Zero Gyroscope");
        }

        @Override
        public void initialize() {
            drivetrain.resetGyroAngle(Rotation2.ZERO);
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}
