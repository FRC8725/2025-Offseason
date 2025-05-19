package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    // ---------- Object ---------- //
    private final TalonFX main = new TalonFX(0);
    private final TalonFX follower = new TalonFX(0);
    private final StatusSignal<Current> statorCurrent = this.main.getStatorCurrent();

    private boolean isZeroed = false;

    // ---------- State ---------- //
    private State state = State.Down;
    public enum State {
        Down(0.0);

        private final double value;

        State(double value) {
            this.value = value;    
        }
    }

    public Elevator() {
        this.configMotor();
        this.follower.setControl(new Follower(this.main.getDeviceID(), true));
    }

    // ---------- Config ----------
    public void configMotor() {
        MotorOutputConfigs outputConfig = new MotorOutputConfigs();
        outputConfig
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

        Slot0Configs slot0Config = new Slot0Configs();
        slot0Config.kS = 0.0;
        slot0Config.kV = 0.0;
        slot0Config.kA = 0.0;
        slot0Config.kG = 0.37;
        slot0Config.kP = 70.0;

        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig
            .withMotionMagicAcceleration(14.0)
            .withMotionMagicCruiseVelocity(3.0);

        FeedbackConfigs feedbackConfig = new FeedbackConfigs();
        feedbackConfig.withSensorToMechanismRatio(Constants.Elevator.MECHANISM_RATIO);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withMotorOutput(outputConfig)
            .withMotionMagic(motionMagicConfig)
            .withSlot0(slot0Config)
            .withFeedback(feedbackConfig);
        this.main.getConfigurator().refresh(motorConfig);        
    }

    // ---------- Method ---------- //
    public void setZeroPositon() {
        this.main.setPosition(0.0);
        this.isZeroed = true;
    }
    public void setZeroVoltage() {
        this.main.setVoltage(Constants.Elevator.ZERO_VOLTAGE);
    }

    public void setCoastMode(boolean isCoast) {
        if (isCoast) {
            this.main.setNeutralMode(NeutralModeValue.Coast);
            this.follower.setNeutralMode(NeutralModeValue.Coast);
        } else {
            this.main.setNeutralMode(NeutralModeValue.Brake);
            this.follower.setNeutralMode(NeutralModeValue.Brake);
        }
    }

    @Override
    public void periodic() {
        if (!this.isZeroed) return;
    }

    // ---------- Function ---------- //
    public boolean atSetpoint() {
        return Math.abs(this.main.getPosition().getValueAsDouble() - this.state.value) < Constants.Elevator.TOLERANCE;
    }

    public double getHeight() {
        return this.main.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return this.main.getVelocity().getValueAsDouble();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Voltage", () -> this.main.getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("Raw Velocity", () -> this.getVelocity(), null);
        builder.addDoubleProperty("Raw Acceleration", () -> this.main.getAcceleration().getValueAsDouble(), null);
        builder.addDoubleProperty("Height", () -> this.getHeight(), null);
        builder.addDoubleProperty("MotionMagic Setpoint", () -> this.main.getClosedLoopReference().getValueAsDouble(), null);
        builder.addBooleanProperty("atSetpoint", () -> this.atSetpoint(), null);
        builder.addBooleanProperty("isZeroed", () -> this.isZeroed, null);
    }
}
