package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.SwerveConstants;

public class Swerve extends SubsystemBase {
    // ---------- Object ---------- //
    private final SwerveModule frontLeft = new SwerveModule(
        2, 1, 9,
        false, true,
        -0.114013671875);
    private final SwerveModule frontRight = new SwerveModule(
        4, 3, 10,
        true, true,
        -0.483154296875);
    private final SwerveModule backLeft = new SwerveModule(
        6, 5, 11,
        false, true,
        -0.1982421875);
    private final SwerveModule backRight = new SwerveModule(
        8, 7, 12,
        true, true,
        -0.27685546875);
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        SwerveConstants.KINEMATICS,
        new Rotation2d(this.getGyroAngle()),
        this.getModulePositions(),
        new Pose2d(0.0, 0.0, new Rotation2d(this.getGyroAngle())),
        VecBuilder.fill(0.1, 0.1, 0.1), // Odometry
        VecBuilder.fill(0.9, 0.9, 2.0)); // Vision

    // ---------- Path PID ---------- //
    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    public Swerve() {
        Shuffleboard.getTab("Swerve").add("Front Left", this.frontLeft);
        Shuffleboard.getTab("Swerve").add("Front Right", this.frontRight);
        Shuffleboard.getTab("Swerve").add("Back Left", this.backLeft);
        Shuffleboard.getTab("Swerve").add("Back Right", this.backRight);
        Shuffleboard.getTab("Swerve").add("Subsystem", this);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // ---------- Function ---------- //
    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public double getGyroAngle() {
        return MathUtil.angleModulus(Units.degreesToRadians(this.gyro.getAngle()));
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    // ---------- Method ---------- //
    public void driveRobotRelative(ChassisSpeeds speeds) {
        ChassisSpeeds discretizeSpeeds = ChassisSpeeds.discretize(speeds, Robot.kDefaultPeriod);
        SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(discretizeSpeeds);
        this.setDesiredState(states);

        // this.poseEstimator.update(new Rotation2d(this.getGyroAngle()), this.getModulePositions()) // TODO: TEST
    }

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = this.getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + this.xController.calculate(pose.getX(), sample.x),
            sample.vy + this.yController.calculate(pose.getY(), sample.y),
            sample.omega + this.headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        this.driveRobotRelative(speeds);
    }

    public void setDesiredState(SwerveModuleState[] states) {
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    public void stopModules() {
        this.driveRobotRelative(new ChassisSpeeds()); // TODO: test
    }

    @Override
    public void periodic() {
        // Add vision measurement...
        this.poseEstimator.update(new Rotation2d(this.getGyroAngle()), this.getModulePositions());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("GyroAngle (Deg)", () -> Units.radiansToDegrees(this.getGyroAngle()), null);
        builder.addDoubleProperty("Vision Angle", () -> this.getPose().getRotation().getDegrees(), null);
        builder.addStringProperty("Pose", () -> this.getPose().toString(), null);
    }
}
