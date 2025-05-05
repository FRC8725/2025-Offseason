package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    public static final double TRACK_WIDTH = Units.inchesToMeters(12.75);
    public static final double TRACK_LENGTH = Units.inchesToMeters(12.75);
    public static final double WHEEL_RADIUS = Units.inchesToMeters(3.95 / 2.0);
    public static final double DRIVE_GEAR_RATIO = 300.0 / 79.0;
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
    public static final int MAX_DRIVE_VOLTAGE = 12;
    public static final int MAX_TURN_VOLTAGE = 8;

    public static final double MAX_SPEED = 2.0;
    public static final double MAX_ACCELERATION = 10.0;
    public static final double MAX_ANGULAR_ACCELERATION = 10.0;

    public static final double DEAD_BAND = 0.05;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(TRACK_LENGTH / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-TRACK_LENGTH / 2.0, -TRACK_LENGTH / 2.0));
}