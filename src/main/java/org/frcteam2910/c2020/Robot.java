package org.frcteam2910.c2020;

import java.io.IOException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.Logger;
import org.frcteam2910.common.robot.UpdateManager;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private static Robot instance = null;

    private static final Logger LOGGER = new Logger(Robot.class);

    private static final byte[] COMPETITION_BOT_MAC_ADDRESS = new byte[]{
            0x00, (byte) 0x80, 0x2f, 0x33, (byte) 0xc4, 0x68
    };
    private static final byte[] PRACTICE_BOT_MAC_ADDRESS = new byte[]{
            0x00, (byte) 0x80, 0x2f, 0x33, (byte) 0xcf, (byte) 0x65
    };

    private static boolean competitionBot;
    private static boolean practiceBot;

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
        robotContainer.getDrivetrainSubsystem().setBrake();

        robotContainer.getDrivetrainSubsystem().setDriveControlMode(DrivetrainSubsystem.DriveControlMode.TRAJECTORY);

        robotContainer.getAutonomousCommand().schedule();
    }

    @Override
    public void teleopInit() {
        robotContainer.getDrivetrainSubsystem().setBrake();
        robotContainer.getDrivetrainSubsystem().setDriveControlMode(DrivetrainSubsystem.DriveControlMode.JOYSTICKS);
        //robotContainer.getClimbElevator().setElevatorMotionMagicPositionAbsolute(27.0);
    }

    @Override
    public void disabledInit(){
        robotContainer.getDrivetrainSubsystem().alignWheels();
        robotContainer.getDrivetrainSubsystem().setCoast();
    }

    @Override
    public void disabledPeriodic() {
        //robotContainer.getVisionSubsystem().setLedMode(Limelight.LedMode.OFF);
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }
}
