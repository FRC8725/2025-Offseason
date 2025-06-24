package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;

public class Swerve extends SubsystemBase {
    private static Swerve SWERVE;
    // ---------- Object ---------- //
    private final SwerveModule frontLeft = new SwerveModule(
        2, 1, 9,
        false, true,
        -0.1123046875);
    private final SwerveModule frontRight = new SwerveModule(
        4, 3, 10,
        true, true,
        -0.4814453125);
    private final SwerveModule backLeft = new SwerveModule(
        6, 5, 11,
        false, true,
        -0.196533203125);
    private final SwerveModule backRight = new SwerveModule(
        8, 7, 12,
        true, true,
        -0.276611328125);
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        Constants.Swerve.KINEMATICS,
        new Rotation2d(this.getGyroAngle()),
        this.getModulePositions(),
        new Pose2d(0.0, 0.0, new Rotation2d(this.getGyroAngle())),
        VecBuilder.fill(0.1, 0.1, 0.1), // Odometry
        VecBuilder.fill(0.9, 0.9, 2.0)); // Vision
    private final StructPublisher<Pose2d> pose = NetworkTableInstance.getDefault()
        .getStructTopic("Component/SwervePose", Pose2d.struct).publish();

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (volts) -> this.setSwerveVoltage(volts.in(Volts)),
            log -> log.motor("frontLeft")
                .voltage(Volts.of(this.frontLeft.getInputVolt()))
                .linearVelocity(MetersPerSecond.of(this.frontLeft.getDriveVolocity()))
                .linearPosition(Meters.of(this.frontLeft.gerDrivePosition())),
            this
        ));

    // ---------- Path PID ---------- //
    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    public Swerve() {
        SWERVE = this;
        Shuffleboard.getTab("Swerve").add("Front Left", this.frontLeft);
        Shuffleboard.getTab("Swerve").add("Front Right", this.frontRight);
        Shuffleboard.getTab("Swerve").add("Back Left", this.backLeft);
        Shuffleboard.getTab("Swerve").add("Back Right", this.backRight);
        Shuffleboard.getTab("Swerve").add("Subsystem", this);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static Swerve getInstance() {
        return SWERVE;
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
        SwerveModuleState[] states = Constants.Swerve.KINEMATICS.toSwerveModuleStates(discretizeSpeeds);
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

    public void resetPose(Pose2d pose) {
        this.poseEstimator.resetPose(pose);
    }

    public void stopModules() {
        this.driveRobotRelative(new ChassisSpeeds()); // TODO: test
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
            this.getQuasistaticForward().withTimeout(5.0),
            this.getDynamicBackward().withTimeout(2.0),
            this.getDynamicForward().withTimeout(2.0));
    }

    @Override
    public void periodic() {
        // Add vision measurement...
        this.poseEstimator.update(new Rotation2d(this.getGyroAngle()), this.getModulePositions());
        this.pose.accept(new Pose2d());

        // SmartDashboard.putData("Quasistatic Forward", this.getQuasistaticForward());
        // SmartDashboard.putData("Dynamic Forward", this.getDynamicForward());
        // SmartDashboard.putData("Quasistatic Backward", this.getQuasistaticBackward());
        // SmartDashboard.putData("Dynamic Backward", this.getDynamicBackward());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("GyroAngle (Deg)", () -> Units.radiansToDegrees(this.getGyroAngle()), null);
        builder.addDoubleProperty("Vision Angle", () -> this.getPose().getRotation().getDegrees(), null);
        builder.addStringProperty("Pose", () -> this.getPose().toString(), null);
    }
}
