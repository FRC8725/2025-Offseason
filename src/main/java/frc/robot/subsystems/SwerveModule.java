package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.constants.SwerveConstants;

public class SwerveModule implements Sendable {
    // ---------- Object ---------- //
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;
    private final CANcoder encoder;
    private final PIDController turnPid;
    private final SimpleMotorFeedforward feedforward;

    // ---------- Value ---------- //
    private double driveVoltage = 0.0;
    private double goalDriveVelocity = 0.0;
    private final double encoderOffset;

    public SwerveModule(
        int driveId, int turnId, int canCoderId,
        boolean driveReverse, boolean turnReverse,
        double encoderOffset
    ) {
        this.driveMotor = new SparkMax(driveId, MotorType.kBrushless);
        this.turnMotor = new SparkMax(turnId, MotorType.kBrushless);
        this.encoder = new CANcoder(canCoderId);

        this.turnPid = new PIDController(3, 0.0, 0.01);
        this.feedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

        this.encoderOffset = encoderOffset;
        this.turnPid.enableContinuousInput(-Math.PI, Math.PI);

        this.configMotors(driveReverse, turnReverse);
        this.resetMotor();
    }

    // ---------- Config ---------- //
    public void configMotors(boolean driveReverse, boolean turnReverse) {
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.encoder.positionConversionFactor(SwerveConstants.DRIVE_GEAR_RATIO);
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.smartCurrentLimit(45);
        driveConfig.inverted(driveReverse);

        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig.encoder.positionConversionFactor(1.0 / SwerveConstants.TURN_GEAR_RATIO);
        turnConfig.idleMode(IdleMode.kCoast);
        turnConfig.inverted(turnReverse);
        turnConfig.smartCurrentLimit(45);
  
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(0.5);
        this.driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.encoder.getConfigurator().apply(canCoderConfig);
    }

    public void resetMotor() {
        this.driveMotor.getEncoder().setPosition(0.0);
        this.turnMotor.getEncoder().setPosition(
            this.encoder.getAbsolutePosition().getValueAsDouble() - this.encoderOffset);
    }

    // ---------- Function ---------- //
    public double getTurnPosition() {
        return MathUtil.angleModulus(this.turnMotor.getEncoder().getPosition() * 2.0 * Math.PI);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveMotor.getEncoder().getPosition() * 2.0 * SwerveConstants.WHEEL_RADIUS * Math.PI,
            new Rotation2d(this.getTurnPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.driveMotor.getEncoder().getVelocity() / 60.0 * 2.0 * SwerveConstants.WHEEL_RADIUS * Math.PI,
            new Rotation2d(this.getTurnPosition()));
    }

    // ---------- Method ---------- //
    public void setDesiredState(SwerveModuleState state) {
        state.optimize(new Rotation2d(this.getTurnPosition()));

        double goalTurnPosition = state.angle.getRadians();
        this.goalDriveVelocity = state.speedMetersPerSecond * Math.cos(this.getTurnPosition() - goalTurnPosition);

        this.driveVoltage = this.feedforward.calculateWithVelocities(this.driveMotor.getEncoder().getVelocity() / 60.0, this.goalDriveVelocity);
        double turnVoltage = this.turnPid.calculate(this.getTurnPosition(), goalTurnPosition);

        this.driveMotor.setVoltage(this.driveVoltage);
        this.turnMotor.setVoltage(turnVoltage);
    }

    public void stop() {
        this.driveMotor.stopMotor();
        this.turnMotor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("/Turn Raw Position (Rot)", () -> this.encoder.getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("/Turn Position (Rad)", () -> this.getTurnPosition(), null);
        builder.addDoubleProperty("/Drive Velocity (Rps)", () -> this.driveMotor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("/Drive Stator Voltage", () -> this.driveMotor.getAppliedOutput(), null);
        builder.addDoubleProperty("/Goal Drive Velocity", () -> this.goalDriveVelocity, null);
        builder.addDoubleProperty("/Goal Drive Voltage", () -> this.driveVoltage, null);
    }
}
