package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveConstants;
import frc.robot.lib.subsystems.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        2, 1, 9,
        false, true,
        "FrontLeft");
    private final SwerveModule frontRight = new SwerveModule(
        4, 3, 10,
        true, true,
        "FrontRight");
    private final SwerveModule backLeft = new SwerveModule(
        6, 5, 11,
        false, true,
        "BackLeft");
    private final SwerveModule backRight = new SwerveModule(
        8, 7, 12,
        true, true,
        "BackRight");
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private final StructPublisher<Pose2d> swervePose = NetworkTableInstance.getDefault()
        .getStructTopic("AdvantageScope/SwervePose", Pose2d.struct).publish();

    Vector<N3> stateStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);
    Vector<N3> visionStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        SwerveConstants.KINEMATICS,
        this.getRotation2d(),
        this.getSwervePosition(),
        new Pose2d(),
        stateStdDevs,
        visionStdDevs);

    public SwerveSubsystem() {
        super("Swerve", false);
    }

    public void drive(double xSpeed, double ySpeed, double rSpeed, boolean fieldOriented) {
        SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(
            fieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed, this.getRotation2d()) :
            new ChassisSpeeds(xSpeed, ySpeed, rSpeed));
        this.setDesiredStates(states);
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        this.setDesiredStates(states);
    }

    @Override
    public void periodic() {
        this.poseEstimator.update(
            this.getRotation2d(), this.getSwervePosition());
        this.swervePose.accept(this.poseEstimator.getEstimatedPosition());
    }

    public double getHeading() {
        return this.gyro.getAngle();
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(Units.degreesToRadians(this.getHeading()));
    }

    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.KINEMATICS.toChassisSpeeds(this.getSwerveState());
    }

    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        this.poseEstimator.resetPose(pose);
    }

    public void resetSwerveEncoders() {
        this.frontLeft.resetEncoder();
        this.frontRight.resetEncoder();
        this.backLeft.resetEncoder();
        this.backRight.resetEncoder();
    }

    public void resetGyro() {
        this.gyro.reset();
    }

    public SwerveModuleState[] getSwerveState() {
        return new SwerveModuleState[] {
            this.frontLeft.getState(),
            this.frontRight.getState(),
            this.backLeft.getState(),
            this.backRight.getState()
        };
    }

    public SwerveModulePosition[] getSwervePosition() {
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    public void setDesiredStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED);
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        System.out.println("A");
        this.poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        this.poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("GyroAngle", this.getHeading());
        SmartDashboard.putString("PoseEstimator", this.poseEstimator.getEstimatedPosition().toString());
    }
}
