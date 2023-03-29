package org.frcteam2910.c2020;

import java.io.IOException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

import org.frcteam2910.c2020.subsystems.Arm.ArmControlMode;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem.DriveControlMode;
import org.frcteam2910.c2020.util.AutonomousChooser.AutonomousMode;
import org.frcteam2910.c2020.util.SideChooser.SideMode;
import org.frcteam2910.common.Logger;
import org.frcteam2910.common.robot.UpdateManager;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private static Robot instance = null;

    private static final Logger LOGGER = new Logger(Robot.class);

    private static final byte[] PRACTICE_BOT_MAC_ADDRESS = new byte[]{
            0x00, (byte) 0x80, 0x2f, 0x33, (byte) 0xc4, 0x68
    };
    private static final byte[] COMPETITION_BOT_MAC_ADDRESS = new byte[]{
            0x00, (byte) 0x80, 0x2f, 0x33, (byte) 0xcf, (byte) 0x65
    };

    private static boolean competitionBot;
    private static boolean practiceBot;
    private static boolean teleopUsed = false;
    private static boolean setSteerMotorsCoast = false;
    private static boolean setArmRotationMotorCoast = false;

    // private static AutonomousMode cachedAutonCommandMode;
    // private static Command cachedAutonCommand;
    // private static SideMode cachedTrajectoriesSide;

    private RobotContainer robotContainer = new RobotContainer();
    private UpdateManager updateManager = new UpdateManager(
            robotContainer.getDrivetrainSubsystem()
    );

    static {
        List<byte[]> macAddresses;
        try {
            macAddresses = getMacAddresses();
        } catch (IOException e) {
            // Don't crash, just log the stacktrace and continue without any mac addresses.
            LOGGER.error(e);
            macAddresses = List.of();
        }

        for (byte[] macAddress : macAddresses) {
            // First check if we are the competition bot
            if (Arrays.compare(COMPETITION_BOT_MAC_ADDRESS, macAddress) == 0) {
                competitionBot = true;
                break;
            }

            // Next check if we are the practice bot
            if (Arrays.compare(PRACTICE_BOT_MAC_ADDRESS, macAddress) == 0) {
                practiceBot = true;
                break;
            }
        }

        if (!competitionBot && !practiceBot) {
            String[] macAddressStrings = macAddresses.stream()
                    .map(Robot::macToString)
                    .toArray(String[]::new);

            SmartDashboard.putStringArray("MAC Addresses", macAddressStrings);
            SmartDashboard.putString("Competition Bot MAC Address", macToString(COMPETITION_BOT_MAC_ADDRESS));
            SmartDashboard.putString("Practice Bot MAC Address", macToString(PRACTICE_BOT_MAC_ADDRESS));

            // If something goes terribly wrong we still want to use the competition bot stuff in competition.
            competitionBot = true;
        }

        SmartDashboard.putBoolean("Competition Bot", competitionBot);
    }

    public Robot() {
        //super(0.01);
        instance = this;
    }

    public static Robot getInstance() {
        return instance;
    }

    public static boolean isCompetitionBot() {
        return competitionBot;
    }

    public static boolean isPracticeBot() {
        return practiceBot;
    }

    /**
     * Gets the MAC addresses of all present network adapters.
     *
     * @return the MAC addresses of all network adapters.
     */
    private static List<byte[]> getMacAddresses() throws IOException {
        List<byte[]> macAddresses = new ArrayList<>();

        Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();

        NetworkInterface networkInterface;
        while (networkInterfaces.hasMoreElements()) {
            networkInterface = networkInterfaces.nextElement();

            byte[] address = networkInterface.getHardwareAddress();
            if (address == null) {
                continue;
            }

            macAddresses.add(address);
        }

        return macAddresses;
    }

    private static String macToString(byte[] address) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < address.length; i++) {
            if (i != 0) {
                builder.append(':');
            }
            builder.append(String.format("%02X", address[i]));
        }
        return builder.toString();
    }

    @Override
    public void robotInit() {
        updateManager.startLoop(0.02);
        robotContainer.getDrivetrainSubsystem().setDriveBrake();
        robotContainer.getDrivetrainSubsystem().setSteerBrake();
        robotContainer.getArm().setArmInchesZero(1.25);
        robotContainer.getArm().setArmDegreesZero(0.0);
        robotContainer.getIntake().setCubeIntakeDeployHome(0.0);
//        PortForwarder.add(5800, "limelight.local", 5800);
//        PortForwarder.add(5801, "limelight.local", 5801);
//        PortForwarder.add(5802, "limelight.local", 5802);
//        PortForwarder.add(5803, "limelight.local", 5803);
//        PortForwarder.add(5804, "limelight.local", 5804);
//        PortForwarder.add(5805, "limelight.local", 5805);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void autonomousInit() {
        teleopUsed = false;
        // robotContainer.getDrivetrainSubsystem().setLimelightOverride(false);

        robotContainer.getDrivetrainSubsystem().setDriveControlMode(DrivetrainSubsystem.DriveControlMode.TRAJECTORY);

        // Would it be better for these to be in robotInit? Probably, once we get the lightgate to zero the arm.
        // robotContainer.getArm().setArmDegreesZero(Constants.ARM_ROTATOR_HOME_DEGREES);

        robotContainer.getAutonomousCommand().schedule();
    }

    @Override
    public void teleopInit() {
        teleopUsed = true;

        CommandScheduler.getInstance().cancelAll();

        robotContainer.getArm().setMotorNeutralMode(NeutralMode.Brake);
        robotContainer.getArm().setRotationHold();
        

        robotContainer.getDrivetrainSubsystem().resetSteerAbsoluteAngle();
        robotContainer.getDrivetrainSubsystem().setDriveCoast();
        robotContainer.getDrivetrainSubsystem().resetGyroAngle(robotContainer.getDrivetrainSubsystem().getPose().rotation);

        // robotContainer.getDrivetrainSubsystem().alignWheels();

        // robotContainer.getArm().setArmInchesZero(Constants.ARM_EXTEND_HOME_INCHES);
        // robotContainer.getArm().setArmDegreesZero(Constants.ARM_HOME_DEGREES);

        robotContainer.updateSide();
        robotContainer.getDrivetrainSubsystem().setDriveCoast();
        robotContainer.getDrivetrainSubsystem().setSteerBrake();
        
        robotContainer.getDrivetrainSubsystem().setDriveControlMode(DrivetrainSubsystem.DriveControlMode.JOYSTICKS);
        //robotContainer.getClimbElevator().setElevatorMotionMagicPositionAbsolute(27.0);
    }

    @Override
    public void disabledInit(){
        if(!setSteerMotorsCoast) {
            robotContainer.getDrivetrainSubsystem().setSteerBrake();
        }
        robotContainer.getDrivetrainSubsystem().alignWheels();
        
        // Safety disable
        List<AutonomousMode> vetted = new ArrayList<AutonomousMode>(
            Arrays.asList(AutonomousMode.SEVEN_FEET,
                        AutonomousMode.S_CURVE,
                        AutonomousMode.THREE_OBJECT_BRIDGE,
                        AutonomousMode.THREE_OBJECT_CLOSE,
                        AutonomousMode.THREE_OBJECT_FAR,
                        AutonomousMode.UP_BRIDGE,
                        AutonomousMode.TO_BRIDGE));

        if(vetted.contains(robotContainer.getAutonomousChooser().getAutonomousModeChooser().getSelected()) || teleopUsed)
        {
            robotContainer.getDrivetrainSubsystem().setDriveCoast();
        }
        else
        {
            // Unsafe command (not been confirmed safe)
            robotContainer.getDrivetrainSubsystem().setDriveBrake();
        }
    }

    @Override
    public void disabledPeriodic() {
        //robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.OFF);

        //Update the side, which should then prompt a re-cache of auton command with the new side.
        // if(robotContainer.getSideChooser().getSendableChooser().getSelected() != cachedTrajectoriesSide) {
        //     robotContainer.updateSide();
        //     robotContainer.recreateTrajectoriesBasedOnSide();
        //     cachedAutonCommand = robotContainer.getAutonomousCommand();
        // }

        // // Update the auton command without updating the trajectories
        // if(robotContainer.getAutonomousChooser().getAutonomousModeChooser().getSelected() != cachedAutonCommandMode) {
        //     cachedAutonCommand = robotContainer.getAutonomousCommand();
        // }
    }

    @Override
    public void testInit() {
        boolean tilted = robotContainer.getDrivetrainSubsystem().getPitchDegreesOffLevel() > 15
                        || robotContainer.getDrivetrainSubsystem().getRollDegreesOffLevel() > 15;
        // Helpful development "unlock steer motors while tilted over" feature
        robotContainer.getDrivetrainSubsystem().setDriveControlMode(DriveControlMode.HOLD);
        robotContainer.getArm().setRotationControlMode(ArmControlMode.MANUAL);
        
        if(!setSteerMotorsCoast) {
            if(tilted){
                robotContainer.getDrivetrainSubsystem().setSteerCoast();
                setSteerMotorsCoast = true;
            }
        }
        else
        {
            robotContainer.getDrivetrainSubsystem().setSteerBrake();
            setSteerMotorsCoast = false;
        }

        if(!setArmRotationMotorCoast && !tilted)
        {
            // We're upright, it's okay to set the arm to coast
            robotContainer.getArm().setMotorNeutralMode(NeutralMode.Coast);
            setArmRotationMotorCoast = true;
        }
        else {
            robotContainer.getArm().setMotorNeutralMode(NeutralMode.Brake);
            setArmRotationMotorCoast = false;
        }
    }

    @Override
    public void testPeriodic() {
    }
}
