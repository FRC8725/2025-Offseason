package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import com.ctre.phoenix6.SignalLogger;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;

public class Swerve extends SubsystemBase {
    private static Swerve SWERVE;
    // ---------- Object ---------- //
    private final SwerveModule frontLeft = new SwerveModule(
        2, 1, 9,
        false, true,
        0.05322265625);
    private final SwerveModule frontRight = new SwerveModule(
        4, 3, 10,
        true, true,
        0.3681640625);
    private final SwerveModule backLeft = new SwerveModule(
        6, 5, 11,
        false, true,
        -0.245361328125);
    private final SwerveModule backRight = new SwerveModule(
        8, 7, 12,
        true, true,
        -0.3212890625);
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        Constants.Swerve.KINEMATICS,
        new Rotation2d(this.getGyroAngle()),
        this.getModulePositions(),
        new Pose2d(0.0, 0.0, new Rotation2d(this.getGyroAngle())),
        VecBuilder.fill(0.1, 0.1, 0.1), // Odometry
        VecBuilder.fill(0.9, 0.9, 2.0)); // Vision
    private final StructPublisher<Pose2d> pose = NetworkTableInstance.getDefault()
        .getStructTopic("Component/SwervePose", Pose2d.struct).publish();

    private Boolean[] probablyScoredPoses = new Boolean[24 * 4];
    public boolean isAligned = false;
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            (state) -> SignalLogger.writeString("frontLeft", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> this.setSwerveVoltage(volts.in(Volts)),
            null,
            this
        ));

    // ---------- Path PID ---------- //
    private final PIDController xController = new PIDController(12.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(12.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(5.0, 0.0, 0.0);

    public Swerve() {
        SWERVE = this;
        Shuffleboard.getTab("Swerve").add("Front Left", this.frontLeft);
        Shuffleboard.getTab("Swerve").add("Front Right", this.frontRight);
        Shuffleboard.getTab("Swerve").add("Back Left", this.backLeft);
        Shuffleboard.getTab("Swerve").add("Back Right", this.backRight);
        Shuffleboard.getTab("Swerve").add("Subsystem", this);

        for (int i = 0; i < this.probablyScoredPoses.length; i++) {
            this.probablyScoredPoses[i] = false;
        }
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static Swerve getInstance() {
        return SWERVE;
    }

    // ---------- Function ---------- //
    public Pose2d getPose() {
        return Robot.isSimulation() ? this.simPose : this.poseEstimator.getEstimatedPosition();
    }

    public Optional<Map.Entry<Integer, Pose2d>> getClosestFudgedScoringPose() {
        ArrayList<Pose2d> poses = Robot.isRedAlliance ?
            Constants.Field.RED_SCORING_POSES :
            Constants.Field.BLUE_SCORING_POSES;

        return IntStream.range(0, poses.size())
            .mapToObj(i -> Map.entry(i, poses.get(i)))
            .filter(entry -> entry.getValue().getTranslation().getDistance(this.getPose().getTranslation()) < Constants.Swerve.MAX_NODE_DISTANCE)
            .min(Comparator.comparingDouble(entry -> {
                int index = entry.getKey();
                Pose2d pose = entry.getValue();
                double fudge = this.wasPoseScored(index) ? Constants.Swerve.ALREADY_SCORED_BADNESS : 0.0;
                return fudge + this.score(pose);
            }));
    }

    public Pose2d getClosestAlgaeGrabPose() {
        ArrayList<Pose2d> poses = Robot.isRedAlliance ? 
            Constants.Field.RED_AGLAE_GRABBING_POSE : 
            Constants.Field.BLUE_AGLAE_GRABBING_POSE;
        poses = new ArrayList<>(
            poses.stream()
            .filter(pose -> pose.getTranslation()
                .getDistance(this.getPose().getTranslation()) < Constants.Swerve.MAX_NODE_DISTANCE)
            .collect(Collectors.toList()));
        
        return poses.stream()
            .min(Comparator.comparingDouble(this::score))
            .orElse(null);
    }

    public Pose2d getClosestThroughScoringPose() {
        ArrayList<Pose2d> poses = Robot.isRedAlliance ? 
            Constants.Field.RED_TROUGH_SCORING_POSES :
            Constants.Field.BLUE_TROUGH_SCORING_POSES;
        poses = new ArrayList<>(poses.stream()
            .filter(pose -> pose.getTranslation()
                .getDistance(this.getPose().getTranslation()) < Constants.Swerve.MAX_NODE_DISTANCE)
            .collect(Collectors.toList()));

        return poses.stream()
            .min(Comparator.comparingDouble(this::score))
            .orElse(null);
    }

    public boolean withinTolerance(Translation2d t) {
        return this.getPose().getTranslation().getDistance(t) < Constants.Swerve.ALIGNMENT_TOLERANCE;
    }

    public boolean wasPoseScored(int index) {
        return this.probablyScoredPoses[index * 4 + SuperStructure.getInstance().input.wantedScoringLevel.index];
    }

    public double getGyroAngle() {
        return MathUtil.angleModulus(Units.degreesToRadians(-this.gyro.getAngle()));
    }

    public double score(Pose2d pose) {
        double translation = pose.getTranslation().getDistance(this.getPose().getTranslation());
        double rotation = Math.abs(pose.getRotation().minus(this.getPose().getRotation()).getRadians());

        return Constants.Swerve.ALIGN_TRANSLATION_WEIGHT * translation +
            Constants.Swerve.ALIGN_ANGLE_WEIGHT * rotation;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    public boolean atGoodScoringDistance() {
        if (!Robot.isOnRedSide) {
            return this.getPose().getX() > Constants.Field.BLUE_BARGE_SCORING_X - 0.1 &&
                this.getPose().getX() < Constants.Field.BLUE_BARGE_SCORING_X + 0.1;
        } else {
            return this.getPose().getX() > Constants.Field.RED_BARGE_SCORING_X - 0.1 &&
                this.getPose().getX() < Constants.Field.RED_BARGE_SCORING_X + 0.1;
        }
    }

    // ---------- Method ---------- //
    public void driveRobotRelative(ChassisSpeeds speeds) {
        ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getPose().getRotation());
        ChassisSpeeds discretizeSpeeds = ChassisSpeeds.discretize(relativeSpeeds, Robot.kDefaultPeriod);
        SwerveModuleState[] states = Constants.Swerve.KINEMATICS.toSwerveModuleStates(discretizeSpeeds);
        this.setDesiredState(states);
        if (Robot.isSimulation()) this.simDriveRobotRelative(discretizeSpeeds);
        this.poseEstimator.update(new Rotation2d(this.getGyroAngle()), this.getModulePositions());
    }

    public void followSample(SwerveSample sample) {
        Pose2d pose = this.getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + this.xController.calculate(pose.getX(), sample.x),
            sample.vy + this.yController.calculate(pose.getY(), sample.y),
            sample.omega + this.headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // speeds.vxMetersPerSecond *= (Robot.isSimulation() ? -1.0 : 1.0);
        // speeds.vyMetersPerSecond *= (Robot.isSimulation() ? -1.0 : 1.0);
        speeds.omegaRadiansPerSecond *= (Robot.isSimulation() ? -1.0 : 1.0);

        this.driveRobotRelative(speeds);
    }

    public void followPose(Pose2d goalPose) {
        Pose2d currentPose = this.getPose();
        ChassisSpeeds speeds = new ChassisSpeeds(
            this.xController.calculate(currentPose.getX(), goalPose.getX()),
            this.yController.calculate(currentPose.getY(), goalPose.getY()),
            this.headingController.calculate(currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians()));
       
        // speeds.vxMetersPerSecond *= (Robot.isSimulation() ? -1.0 : 1.0);
        // speeds.vyMetersPerSecond *= (Robot.isSimulation() ? -1.0 : 1.0);
        speeds.omegaRadiansPerSecond *= (Robot.isSimulation() ? -1.0 : 1.0);

        this.driveRobotRelative(speeds);
    }

    public void setDesiredState(SwerveModuleState[] states) {
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    public void resetPose(Pose2d pose) {
        this.poseEstimator.resetPose(pose);
        this.simPose = pose;
    }

    public void resetYaw(double angle) {
        this.gyro.setAngleAdjustment(Units.radiansToDegrees(MathUtil.angleModulus(angle)));
    }

    public void markPoseScored() {
        if (!this.getClosestFudgedScoringPose().isPresent()) return; // TODO 7/30
        int index = this.getClosestFudgedScoringPose().get().getKey();
        this.probablyScoredPoses[index * 4 + SuperStructure.getInstance().input.wantedScoringLevel.index] = true;
    }

    public void stopModules() {
        this.driveRobotRelative(new ChassisSpeeds());
    }

    public void setSwerveVoltage(double voltage) {
        this.frontLeft.setDriveVoltage(voltage);
        this.frontRight.setDriveVoltage(voltage);
        this.backLeft.setDriveVoltage(voltage);
        this.backRight.setDriveVoltage(voltage);
    }

    public Command getQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command getDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command getQuasistaticBackward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command getDynamicBackward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdTest() {
        return Commands.sequence(
            this.getQuasistaticBackward().withTimeout(5.0),
            new WaitCommand(1.0),
            this.getQuasistaticForward().withTimeout(5.0),
            new WaitCommand(1.0),
            this.getDynamicBackward().withTimeout(2.0),
            new WaitCommand(1.0),
            this.getDynamicForward().withTimeout(2.0));
    }

    @Override
    public void periodic() {
        // Add vision measurement...
        this.poseEstimator.update(new Rotation2d(this.getGyroAngle()), this.getModulePositions());
        this.pose.accept(this.getPose());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("GyroAngle (Deg)", () -> Units.radiansToDegrees(this.getGyroAngle()), null);
        builder.addDoubleProperty("Vision Angle", () -> this.getPose().getRotation().getDegrees(), null);
        builder.addStringProperty("Pose", () -> this.getPose().toString(), null);
        builder.addBooleanProperty("atGoodDistance", () -> this.atGoodScoringDistance(), null);
    }

    // ---------- Simulation ---------- //
    private Pose2d simPose = new Pose2d();
    private double simAngle = 0.0;

    public void simDriveRobotRelative(ChassisSpeeds speeds) {        
        if (speeds.vxMetersPerSecond != 0.0 || speeds.vyMetersPerSecond != 0.0 || speeds.omegaRadiansPerSecond != 0.0) {
            double simPoseX = this.simPose.getX() + -speeds.vxMetersPerSecond * 0.02;
            double simPoseY = this.simPose.getY() + -speeds.vyMetersPerSecond * 0.02;
            this.simAngle -= speeds.omegaRadiansPerSecond * 0.02;
            this.simPose = new Pose2d(simPoseX, simPoseY, new Rotation2d(this.simAngle));
        }
    }
}
