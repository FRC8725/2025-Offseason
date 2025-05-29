package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public final class Elevator {
        public static final double GEAR_RATIO = 4.0;
        public static final double SPOOL_RADIUS = Units.inchesToMeters(0.75);
        public static final double MECHANISM_RATIO = GEAR_RATIO / (2.0 * SPOOL_RADIUS * Math.PI);
        public static final double TOLERANCE = 0.01;
        public static final double ZERO_VOLTAGE = 0.3;
    }

    public final class Arm {
        public static final double IDEL_CURRENT_DRAW = 10.0; // TODO
        public static final double CURRENT_DRAW = 15.0; // TODO
    }

    public final class Vision {
        public static final Transform3d FRONT_LEFT = new Transform3d(
            0.0, 0.0, 0.0, new Rotation3d());
    }

    public final class Field {
        public static final double FIELD_X_SIZE = 17.548249;
        public static final double FIELD_Y_SIZE = 8.0518;
        
    }

    public final class Swerve {
        public static final double TRACK_WIDTH = Units.inchesToMeters(12.75);
        public static final double TRACK_LENGTH = Units.inchesToMeters(12.75);
        public static final double WHEEL_RADIUS = Units.inchesToMeters(3.95 / 2.0);
        public static final double DRIVE_GEAR_RATIO = 300.0 / 49.0;
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

        public static final double MAX_MOTOR_RPM = 5500.0;
        public static final double MAX_VELOCITY = (MAX_MOTOR_RPM / 60.0) / DRIVE_GEAR_RATIO * 2.0 * WHEEL_RADIUS * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (TRACK_WIDTH * Math.sqrt(2.0)); // TODO: Research HOW

        public static final double DEAD_BAND = 0.05;

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(TRACK_LENGTH / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-TRACK_LENGTH / 2.0, -TRACK_LENGTH / 2.0));
    }
}
