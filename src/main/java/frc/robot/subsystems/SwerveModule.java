package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;

public class SwerveModule implements Sendable {
    // ---------- Object ---------- //
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder encoder;
    private final PIDController turnPid;
    private final SimpleMotorFeedforward feedforward;

    // ---------- Value ---------- //
    private double driveVoltage = 0.0;
    private double goalDriveVelocity = 0.0;
    private double goalTurnPosition = 0.0;
    private final double encoderOffset;

    public SwerveModule(
        int driveId, int turnId, int canCoderId,
        boolean driveReverse, boolean turnReverse,
        double encoderOffset
    ) {
        this.driveMotor = new TalonFX(driveId);
        this.turnMotor = new TalonFX(turnId);
        this.encoder = new CANcoder(canCoderId);

        this.turnPid = new PIDController(3.6, 0.1, 0.0);
        this.feedforward = new SimpleMotorFeedforward(0.2187, 0.0019282, 0.00021717);

        this.encoderOffset = encoderOffset;
        this.turnPid.enableContinuousInput(-Math.PI, Math.PI);

        this.configMotors(driveReverse, turnReverse);
        this.resetMotor();
    }

    // ---------- Config ---------- //
    public void configMotors(boolean driveReverse, boolean turnReverse) {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Feedback
            .withSensorToMechanismRatio(Constants.Swerve.DRIVE_GEAR_RATIO);
        driveConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(driveReverse ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
        driveConfig.CurrentLimits
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(45.0)
            .withStatorCurrentLimitEnable(true) // TODO: what did CTRE do with these?
            .withStatorCurrentLimit(80.0);

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.Feedback
            .withSensorToMechanismRatio(Constants.Swerve.TURN_GEAR_RATIO);
        turnConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(turnReverse ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
        turnConfig.CurrentLimits
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(45.0)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(80.0);

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(0.5)
            .withMagnetOffset(this.encoderOffset);

        this.driveMotor.getConfigurator().apply(driveConfig);
        this.turnMotor.getConfigurator().apply(turnConfig);
        this.encoder.getConfigurator().apply(canCoderConfig);
    }

    public void resetMotor() {
        this.driveMotor.setPosition(0.0);
        this.turnMotor.setPosition(this.encoder.getAbsolutePosition().getValueAsDouble());
    }

    // ---------- Function ---------- //
    public double getTurnPosition() {
        double position = this.encoder.getAbsolutePosition().getValueAsDouble();
        position %= 1.0;
        return Units.rotationsToRadians(position > 0.5 ? position - 1.0 : position);
    }

    public double getInputVolt() {
        return this.driveMotor.getMotorVoltage().getValueAsDouble();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveMotor.getPosition().getValueAsDouble() * 2.0 * Constants.Swerve.WHEEL_RADIUS * Math.PI,
            new Rotation2d(this.getTurnPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            this.driveMotor.getVelocity().getValueAsDouble() * 2.0 * Constants.Swerve.WHEEL_RADIUS * Math.PI,
            new Rotation2d(this.getTurnPosition()));
    }

    public double getDriveVolocity() {
        return this.driveMotor.getVelocity().getValueAsDouble();
    }

    public double gerDrivePosition() {
        return this.driveMotor.getPosition().getValueAsDouble();
    }

    // ---------- Method ---------- //
    public void setDesiredState(SwerveModuleState state) {
        if (state.speedMetersPerSecond < 0.001) {
            this.stop();
            return;
        }
        state.optimize(Rotation2d.fromRadians(this.getTurnPosition()));

        this.goalTurnPosition = state.angle.getRadians();
        
        this.goalDriveVelocity = state.speedMetersPerSecond * Math.cos(this.getTurnPosition() - this.goalTurnPosition);
        this.driveVoltage = this.feedforward.calculate(this.getDriveVolocity(), this.driveMotor.getAcceleration().getValueAsDouble());
        double turnVoltage = this.turnPid.calculate(this.getTurnPosition(), goalTurnPosition);

        this.driveMotor.setVoltage(this.driveVoltage);
        this.turnMotor.setVoltage(turnVoltage);
    }

    public void setDriveVoltage(double voltage) {
        this.driveMotor.setVoltage(voltage);
    }

    public void stop() {
        this.driveMotor.stopMotor();
        this.turnMotor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("/Turn Raw Position (Rot)", () -> this.encoder.getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("/Turn Position (Rad)", () -> this.getTurnPosition(), null);
        builder.addDoubleProperty("/Drive Velocity (Rps)", () -> this.getDriveVolocity(), null);
        builder.addDoubleProperty("/Drive Stator Voltage", () -> this.getInputVolt(), null);
        builder.addDoubleProperty("/Goal Drive Velocity", () -> this.goalDriveVelocity, null);
        builder.addDoubleProperty("/Goal Drive Voltage", () -> this.driveVoltage, null);
        builder.addDoubleProperty("Goal Turn Position", () -> this.goalTurnPosition, null);
    }
}
