package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.math.MathUtils;

public final class Constants {
    public final class Elevator {
        // Mechanism
        public static final double GEAR_RATIO = 4.0;
        public static final double SPOOL_RADIUS = Units.inchesToMeters(0.75);
        public static final double MECHANISM_RATIO = GEAR_RATIO / (2.0 * SPOOL_RADIUS * Math.PI);

        // Zero
        public static final double ZERO_VOLTAGE = 0.28;
        public static final double ZERO_MIN_CURRENT = 1.7; // Amps

        // Height
        public static final double SAFE_HEIGHT = Units.inchesToMeters(32.5);
        public static final double MAX_EXTENSION = Units.inchesToMeters(49.25);

        // Tolerance
        public static final double TOLERANCE = 0.01;
        public static final double LAZIER_TOLERANCE = 0.03;
    }

    public final class Arm {
        // Mechanism
        public static final double GEAR_RATIO = 6272.0 / 81.0;
        public static final double POSITION_DEPENDENT_KG = 0.29;
        public static final double CORAL_CENTER_OFFSET = Units.inchesToMeters(9.5); // TODO
        public static final double ENCODER_OFFSET_ROTATION = 0.5197092879927322;

        // Detect has object
        public static final double IDEL_CURRENT_DRAW = 10.0;
        public static final double CURRENT_DRAW = 15.0;

        // Safe
        public static final double SAFE_INSIDE_ROBOT_ANGLE = Units.degreesToRadians(40.0);
        public static final double SAFE_DISTANCE_FROM_REEF_CENTER = Units.inchesToMeters(70.0);
        public static final double SAFE_BARGE_DISTANCE = Units.inchesToMeters(50.0);
        public static final double SAFE_PLACEMENT_DISANCE = Units.inchesToMeters(60.0);

        // Limit
        public static final double ALLOWED_OPERATING_MIN_RADIANS = Units.degreesToRadians(-350.0);
        public static final double ALLOWED_OPERATING_MAX_RADIANS = Units.degreesToRadians(350.0);

        // Tolerance
        public static final double TOLERANCE = 0.1;
        public static final double DEADZONE_ANGLE = Units.degreesToRadians(20.0); // TODO
    }

    public final class Intake {
        // Mechanism
        public static final double GEAR_RATIO = 540.0 / 7.0;
        public static final double LASERCAN_DISTANCE = 50.0;

        // Zero
        public static final double ZERO_VOLTAGE = 0.0;
        public static final double ZERO_MIN_CURRENT = 20.0; // Amps

        // Tolerance
        public static final double TOLERANCE = 0.01;
    }

    public final class Vision {
        public static final Transform3d FRONT_LEFT = new Transform3d(
            -0.010313, 0.301234, 0.1922698,
            new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(70.0)));
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

    public final class Swerve {
        // Module
        public static final double TRACK_WIDTH = Units.inchesToMeters(12.75);
        public static final double TRACK_LENGTH = Units.inchesToMeters(12.75);
        public static final double WHEEL_RADIUS = Units.inchesToMeters(3.95 / 2.0);
        public static final double DRIVE_GEAR_RATIO = 57.0 / 7.0;
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

        // Phycical Limit
        public static final double MAX_MOTOR_RPM = 3000.0;
        public static final double MAX_VELOCITY = (MAX_MOTOR_RPM / 60.0) / DRIVE_GEAR_RATIO * 2.0 * WHEEL_RADIUS * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / (TRACK_WIDTH * Math.sqrt(2.0)); // TODO: Research HOW

        // Align Limit
        public static final double MAX_BARGE_ALIGN_TRANSLATION_SPEED = 1.5;
        public static final double MAX_BARGE_ALIGN_ROTAITON_SPEED = 1.5;
        public static final double MAX_ALIGN_TRANSLATION_SPEED = 1.5;

        // Tolerance
        public static final double DEAD_BAND = 0.05;
        public static final double STRATING_TOLERANCE = 0.15;
        public static final double ALIGNMENT_TOLERANCE = 0.04;

        // Align
        public static final double MAX_NODE_DISTANCE = 3.0;
        public static final double ALIGN_TRANSLATION_WEIGHT = 5.0;
        public static final double ALIGN_ANGLE_WEIGHT = 2.7;
        public static final double ALREADY_SCORED_BADNESS = 0.5 + Units.inchesToMeters(12.9375) * ALIGN_ANGLE_WEIGHT * (1.0 - 2.0 * 0.3);

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(TRACK_LENGTH / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-TRACK_LENGTH / 2.0, -TRACK_LENGTH / 2.0));
    }

    public final class Field {
        // Field size
        public static final double FIELD_X_SIZE = 17.548249;
        public static final double FIELD_Y_SIZE = 8.0518;

        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(Units.inchesToMeters(176.75), Units.inchesToMeters(158.5));
        
        // Safe
        public static final double SAFE_WALL_DISTANCE = 1.0;

        // Distance
        public static final double ROBOT_REEF_CENTER_DISTANCE = Units.inchesToMeters(53.0);
        public static final double REEF_BRANCE_OFFSET_DISTANCE = Units.inchesToMeters(12.9375 / 2.0);
        public static final double TROUGH_OFFSET_DISTANCE = Units.inchesToMeters(14.5 / 2.0);

        // Score pose
        public static final double BLUE_BARGE_SCORING_X = 7.0;
        public static final double RED_BARGE_SCORING_X = FIELD_X_SIZE - BLUE_BARGE_SCORING_X;

        // Poses
        public static final ArrayList<Pose2d> BLUE_SCORING_POSES = getBlueScoringPoses();
        public static final ArrayList<Pose2d> BLUE_TROUGH_SCORING_POSES = getBlueTroughScoringPoses();
        public static final ArrayList<Pose2d> BLUE_AGLAE_GRABBING_POSE = getBlueAlgaeGrabbingPose();

        public static final ArrayList<Pose2d> RED_SCORING_POSES = 
            new ArrayList<>(BLUE_SCORING_POSES.stream()
                .map(pose -> MathUtils.mirror(pose))
                .collect(Collectors.toList()));
        public static final ArrayList<Pose2d> RED_TROUGH_SCORING_POSES = 
            new ArrayList<>(BLUE_TROUGH_SCORING_POSES.stream()
                .map(pose -> MathUtils.mirror(pose))
                .collect(Collectors.toList()));
        public static final ArrayList<Pose2d> RED_AGLAE_GRABBING_POSE =
            new ArrayList<>(BLUE_AGLAE_GRABBING_POSE.stream()
                .map(pose -> MathUtils.mirror(pose))
                .collect(Collectors.toList()));
    }    

    public final class CoralStation {
        public static final Pose2d RIGHT_CENTER_FACE = new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90.0));
        public static final Pose2d LEFT_CENTER_FACE = new Pose2d(
            RIGHT_CENTER_FACE.getX(),
            Field.FIELD_Y_SIZE - RIGHT_CENTER_FACE.getY(),
            Rotation2d.fromRadians(-RIGHT_CENTER_FACE.getRotation().getRadians()));
        
    }

    public static final List<Pair<Double, Double>> armElevatorPairs = List.of(
        new Pair<>(Units.degreesToRadians(7.0), 0.0),
        new Pair<>(Units.degreesToRadians(12.0), Units.inchesToMeters(1.0)),
        new Pair<>(Units.degreesToRadians(18.0), Units.inchesToMeters(2.0)),
        new Pair<>(Units.degreesToRadians(22.0), Units.inchesToMeters(3.0)),
        new Pair<>(Units.degreesToRadians(26.0), Units.inchesToMeters(4.0)),
        new Pair<>(Units.degreesToRadians(30.0), Units.inchesToMeters(5.0)),
        new Pair<>(Units.degreesToRadians(35.0), Units.inchesToMeters(6.0)),
        new Pair<>(Units.degreesToRadians(45.0), Units.inchesToMeters(7.0)),
        new Pair<>(Units.degreesToRadians(51.0), Units.inchesToMeters(8.0)),
        new Pair<>(Units.degreesToRadians(58.0), Units.inchesToMeters(9.0)),
        new Pair<>(Units.degreesToRadians(63.0), Units.inchesToMeters(10.0)),
        new Pair<>(Units.degreesToRadians(68.0), Units.inchesToMeters(11.0)), // TODO 7/3
        new Pair<>(Units.degreesToRadians(77.0), Units.inchesToMeters(13.0)),
        new Pair<>(Units.degreesToRadians(95.0), Units.inchesToMeters(16.0)),
        new Pair<>(Units.degreesToRadians(105.0), Units.inchesToMeters(18.0)),
        new Pair<>(Units.degreesToRadians(115.0), Units.inchesToMeters(20.0)),
        new Pair<>(Units.degreesToRadians(120.0), Units.inchesToMeters(23.0)),
        new Pair<>(Units.degreesToRadians(125.0), Units.inchesToMeters(25.0)),
        new Pair<>(Units.degreesToRadians(130.0), Units.inchesToMeters(27.0)),
        new Pair<>(Units.degreesToRadians(135.0), Units.inchesToMeters(29.0)),
        new Pair<>(Units.degreesToRadians(140.0), Units.inchesToMeters(31.5)),
        new Pair<>(Units.degreesToRadians(180.0), Elevator.SAFE_HEIGHT));

    public static final List<Pair<Double, Double>> armInterpolationIntakeDown = List.of(
        new Pair<>(Units.degreesToRadians(88.0), 0.0),
        new Pair<>(Units.degreesToRadians(105.0), Units.inchesToMeters(5.0)),
        new Pair<>(Units.degreesToRadians(113.0), Units.inchesToMeters(8.0)),
        new Pair<>(Units.degreesToRadians(117.0), Units.inchesToMeters(13.0)),
        new Pair<>(Units.degreesToRadians(131.0), Units.inchesToMeters(18.0)),
        new Pair<>(Units.degreesToRadians(137.0), Units.inchesToMeters(21.0)),
        new Pair<>(Units.degreesToRadians(180.0), Units.inchesToMeters(23.0)));

    public static ArrayList<Pose2d> getBlueScoringPoses() {
        ArrayList<Pose2d> poseArray = new ArrayList<>();
        // Blue scoring poses
        for (int angle = 0; angle < 360; angle += 60) {
            for (int direction : new int[]{1, -1}) {
                for (int side : new int[]{1, -1}) {
                    Pose2d pose = new Pose2d(Field.BLUE_REEF_CENTER, Rotation2d.kZero)
                        .plus(new Transform2d(
                            new Translation2d(
                                -Field.ROBOT_REEF_CENTER_DISTANCE,
                                new Rotation2d(Units.degreesToRadians(angle))),
                            Rotation2d.kZero))
                        .plus(new Transform2d(
                            new Translation2d(
                                direction * Field.REEF_BRANCE_OFFSET_DISTANCE,
                                new Rotation2d(Units.degreesToRadians(angle + 90.0))),
                            new Rotation2d(Units.degreesToRadians(angle))))
                        .plus(new Transform2d(
                            0.0, side * Arm.CORAL_CENTER_OFFSET, new Rotation2d(-side * Math.PI / 2.0)));
                    
                    poseArray.add(pose);
                }
            }
        }

        return poseArray;
    }

    public static ArrayList<Pose2d> getBlueTroughScoringPoses() {
        ArrayList<Pose2d> poseArray = new ArrayList<>();
        // Blue trough poses
        for (int angle = 0; angle < 360; angle += 60) {
            for (int direction : new int[]{1, -1}) {
                Rotation2d angleRot = new Rotation2d(Units.degreesToRadians(angle));
                Rotation2d anglePlus90Rot = new Rotation2d(Units.degreesToRadians(angle + 90.0));

                Pose2d pose = new Pose2d(Field.BLUE_REEF_CENTER, Rotation2d.kZero)
                    .plus(new Transform2d(
                        new Translation2d(-Field.ROBOT_REEF_CENTER_DISTANCE, angleRot),
                        Rotation2d.kZero))
                    .plus(new Transform2d(
                        new Translation2d(direction * Field.TROUGH_OFFSET_DISTANCE, anglePlus90Rot),
                        angleRot));

                    poseArray.add(pose);
            }
        }

        return poseArray;
    }

    public static ArrayList<Pose2d> getBlueAlgaeGrabbingPose() {
        ArrayList<Pose2d> poseArray = new ArrayList<>();
        // Blue algae poses
        for (int angle = 0; angle < 360; angle += 60) {
            for (int side : new int[]{1, -1}) {
                Rotation2d angleRot = new Rotation2d(Units.degreesToRadians(angle));

                Pose2d pose = new Pose2d(Field.BLUE_REEF_CENTER, Rotation2d.kZero)
                    .plus(new Transform2d(
                        new Translation2d(-Field.ROBOT_REEF_CENTER_DISTANCE + 0.05, angleRot),
                        Rotation2d.kZero))
                    .plus(new Transform2d(
                        0.0, side * Arm.CORAL_CENTER_OFFSET, new Rotation2d(-side * Math.PI / 2.0)));

                poseArray.add(pose);
            }
        }

        return poseArray;
    }

    public static ArrayList<Pose3d> getCoralSimualtionScoreLocation() {
        ArrayList<Pose3d> poseArray = new ArrayList<>();
        double[] levelHeight = new double[]{
            Units.inchesToMeters(18.95),
            Units.inchesToMeters(29.3),
            Units.inchesToMeters(45.3),
            1.75};
        double[] levelDistance = new double[]{
            Units.inchesToMeters(30.0),
            Units.inchesToMeters(28.0),
            Units.inchesToMeters(28.0),
            Units.inchesToMeters(30.5)
        };

        
        for (int angle = 0; angle < 360; angle += 60) { // Plane
            for (int direction : new int[]{1, -1}) {
                for (@SuppressWarnings("unused") int side : new int[]{1, -1}) {
                    for (int i = 0; i < 4; i++) { // Floor
                        Pose2d pose = new Pose2d(Field.BLUE_REEF_CENTER, Rotation2d.kZero)
                            .plus(new Transform2d(
                                new Translation2d(
                                    -levelDistance[i],
                                    new Rotation2d(Units.degreesToRadians(angle))),
                                Rotation2d.kZero))
                            .plus(new Transform2d(
                                new Translation2d(
                                    direction * Field.REEF_BRANCE_OFFSET_DISTANCE,
                                    new Rotation2d(Units.degreesToRadians(angle + 90.0))),
                                Rotation2d.kZero));

                        Rotation3d coralRot = new Rotation3d(
                            0.0,
                            (i == 1 || i == 2 ? Units.degreesToRadians(35.0) :
                            (i == 3 ? Units.degreesToRadians(90.0) : 0.0)),
                            Units.degreesToRadians(angle + (i == 0 ? 90.0 : 0.0)));
                        
                        poseArray.add(new Pose3d(pose)
                            .plus(new Transform3d(
                                0.0, 0.0, levelHeight[i], coralRot)));
                    }
                }
            }
        }

        return poseArray;
    }
}
