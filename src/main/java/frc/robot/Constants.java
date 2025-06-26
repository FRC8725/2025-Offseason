package frc.robot;

import java.util.List;

import edu.wpi.first.math.Pair;
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
        public static final double ZERO_VOLTAGE = 0.29;
        public static final double SAFE_HEIGHT = Units.inchesToMeters(34.0);
        public static final double LAZIER_SETPOINT_THRESHOLD = 0.03;
    }

    public final class Arm {
        public static final double GEAR_RATIO = 224.0 / 3.0;
        public static final double IDEL_CURRENT_DRAW = 10.0; // TODO
        public static final double CURRENT_DRAW = 15.0; // TODO
        public static final double SAFE_INSIDE_ROBOT_ANGLE = Units.degreesToRadians(32.5);
        public static final double ALLOWED_OPERATING_MIN_RADIANS = Units.degreesToRadians(-350.0);
        public static final double ALLOWED_OPERATING_MAX_RADIANS = Units.degreesToRadians(350.0);
        public static final double DEADZONE_ANGLE = Units.degreesToRadians(20.0); // TODO
        public static final double SAFE_DISTANCE_FROM_REEF_CENTER = Units.inchesToMeters(70.0);
        public static final double POSITION_DEPENDENT_KG = 0.29;
        public static final double ENCODER_OFFSET_ROTATION = 0.0;
        public static final double SETPOINT_THRESHOLD = 0.0;
        public static final double SAFE_BARGE_DISTANCE = Units.inchesToMeters(50.0);
        public static final double SAFE_PLACEMENT_DISANCE = Units.inchesToMeters(60.0);
    }

    public final class Vision {
        public static final Transform3d FRONT_LEFT = new Transform3d(
            -0.010313, 0.301234, 0.1922698,
            new Rotation3d(0.0, Units.degreesToRadians(20.0), Units.degreesToRadians(70.0)));
        public static final Transform3d BACK_LEFT = new Transform3d(
            -0.087195, 0.301234, 0.1922698,
            new Rotation3d(0.0, Units.degreesToRadians(20.0), Units.degreesToRadians(125.0)));
        public static final Transform3d FRONT_RIGHT = new Transform3d(
            -0.010313, -0.301234, 0.1922698,
            new Rotation3d(0.0, Units.degreesToRadians(20.0), Units.degreesToRadians(-70.0)));
        public static final Transform3d BACK_RIGHT = new Transform3d(
            -0.087195, -0.301234, 0.1922698,
            new Rotation3d(0.0, Units.degreesToRadians(20.0), Units.degreesToRadians(-125.0)));
    }

    public final class Field {
        public static final double FIELD_X_SIZE = 17.548249;
        public static final double FIELD_Y_SIZE = 8.0518;
        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(Units.inchesToMeters(176.75), Units.inchesToMeters(158.5));
        public static final double BLUE_BARGE_SCORING_X = 7.0;
        public static final double RED_BARGE_SCORING_X = FIELD_X_SIZE - BLUE_BARGE_SCORING_X;
        public static final double SAFE_WALL_DISTANCE = 1.0;
    }

    public final class Swerve {
        public static final double TRACK_WIDTH = Units.inchesToMeters(12.75);
        public static final double TRACK_LENGTH = Units.inchesToMeters(12.75);
        public static final double WHEEL_RADIUS = Units.inchesToMeters(3.95 / 2.0);
        public static final double DRIVE_GEAR_RATIO = 57.0 / 7.0;
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

        public static final double MAX_MOTOR_RPM = 5500.0;
        public static final double MAX_VELOCITY = (MAX_MOTOR_RPM / 60.0) / DRIVE_GEAR_RATIO * 2.0 * WHEEL_RADIUS * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (TRACK_WIDTH * Math.sqrt(2.0)); // TODO: Research HOW

        public static final double DEAD_BAND = 0.05;
        public static final double STRATING_TOLERANCE = 0.15;
        public static final double ALIGNMENT_TOLERANCE = 0.04;

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(TRACK_LENGTH / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-TRACK_LENGTH / 2.0, -TRACK_LENGTH / 2.0));
    }

    public final class Intake {
        public static final double GEAR_RATIO = 160.0 / 3.0;
        public static final double ZERO_VOLTAGE = 0.0;
        public static final double LASER_DISTANCE = 0.0; // TODO
        public static final double TOLERANCE = 0.01;
    }

    public static final List<Pair<Double, Double>> armElevatorPairs = List.of(
        new Pair<>(Units.degreesToRadians(0.0), 0.0),
        new Pair<>(Units.degreesToRadians(5.0), Units.inchesToMeters(2.0)),
        new Pair<>(Units.degreesToRadians(15.0), Units.inchesToMeters(3.0)),
        new Pair<>(Units.degreesToRadians(25.0), Units.inchesToMeters(4.0)),
        new Pair<>(Units.degreesToRadians(30.0), Units.inchesToMeters(5.0)),
        new Pair<>(Units.degreesToRadians(35.0), Units.inchesToMeters(6.0)),
        new Pair<>(Units.degreesToRadians(35.0), Units.inchesToMeters(7.0)),
        new Pair<>(Units.degreesToRadians(40.0), Units.inchesToMeters(8.0)),
        new Pair<>(Units.degreesToRadians(45.0), Units.inchesToMeters(9.0)),
        new Pair<>(Units.degreesToRadians(60.0), Units.inchesToMeters(11.0)),
        new Pair<>(Units.degreesToRadians(83.0), Units.inchesToMeters(13.5)),
        new Pair<>(Units.degreesToRadians(93.0), Units.inchesToMeters(16.0)),
        new Pair<>(Units.degreesToRadians(105.0), Units.inchesToMeters(18.0)),
        new Pair<>(Units.degreesToRadians(115.0), Units.inchesToMeters(20.0)),
        new Pair<>(Units.degreesToRadians(120.0), Units.inchesToMeters(23.0)),
        new Pair<>(Units.degreesToRadians(130.0), Units.inchesToMeters(26.5)),
        new Pair<>(Units.degreesToRadians(135.0), Units.inchesToMeters(28.5)),
        new Pair<>(Units.degreesToRadians(140.0), Units.inchesToMeters(31.5)),
        new Pair<>(Units.degreesToRadians(180.0), Elevator.SAFE_HEIGHT)
    );

    public static final List<Pair<Double, Double>> armInterpolationIntakeDown = List.of();
}
