package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    // ---------- Object ---------- //
    private final TalonFX main = new TalonFX(13);
    private final TalonFX follower = new TalonFX(14);
    private final Follower follower2 = new Follower(this.main.getDeviceID(), true);
    private final StatusSignal<Current> statorCurrent = this.main.getStatorCurrent();
    private final MotionMagicVoltage request = new MotionMagicVoltage(0);

    private static boolean isZeroed = false;

    // ---------- State ---------- //
    public static State state = State.Down;
    public enum State {
        Down(0.0),
        PreHandoff(Units.inchesToMeters(36.0)),
        Handoff(0.0),
        PopcicleHandoff(0.0),
        PreScore(0.0),
        Through(0.0),
        L2(0.0),
        L3(0.0),
        L4(0.0),
        Barge(0.0),
        ScoreL2(0.0),
        ScoreL3(0.0),
        ScoreL4(0.0),
        PostL2(0.0),
        PostL3(0.0),
        HighAglae(0.0),
        LowAglae(0.0),
        SourceIntake(0.0);

        private final double value;

        State(double value) {
            this.value = value;    
        }
    }

    public Elevator() {
        this.configMotor();
        this.setZeroPositon();
        // this.follower.setControl(new Follower(this.main.getDeviceID(), true));
    }

    // ---------- Config ----------
    public void configMotor() {
        MotorOutputConfigs outputConfig = new MotorOutputConfigs();
        outputConfig
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

        Slot0Configs slot0Config = new Slot0Configs();
        slot0Config.kS = 0.0;
        slot0Config.kV = 0.0;
        slot0Config.kA = 0.0;
        slot0Config.kG = 0.3;
        slot0Config.kP = 70.0;

        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig
            .withMotionMagicAcceleration(14.0)
            .withMotionMagicCruiseVelocity(3.0);

        FeedbackConfigs feedbackConfig = new FeedbackConfigs();
        feedbackConfig.SensorToMechanismRatio = Constants.Elevator.MECHANISM_RATIO;

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withMotorOutput(outputConfig)
            .withMotionMagic(motionMagicConfig)
            .withSlot0(slot0Config)
            .withFeedback(feedbackConfig);
        this.main.getConfigurator().apply(motorConfig); 
        this.follower.getConfigurator().apply(motorConfig);  
    }

    // ---------- Method ---------- //
    public void setZeroPositon() {
        this.main.setPosition(0.0);
        isZeroed = true;
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
        if (!isZeroed) return;
        this.main.setControl(this.request.withPosition(state.value));
        this.follower.setControl(this.follower2);
    }

    // ---------- Function ---------- //
    public static boolean isZeroed() {
        return isZeroed;
    }
    
    public boolean atSetpoint() {
        return Math.abs(this.main.getPosition().getValueAsDouble() - state.value) < Constants.Elevator.TOLERANCE;
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
        builder.addBooleanProperty("isZeroed", () -> isZeroed, null);
        builder.addStringProperty("State", () -> state.toString(), null);
    }

    // ---------- Simulation ---------- //
    private final StructPublisher<Pose3d> stageComponent = NetworkTableInstance.getDefault()
        .getStructTopic("Component/StageComponent", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> carriageComponent = NetworkTableInstance.getDefault()
        .getStructTopic("Component/CarriageComponent", Pose3d.struct).publish();
    private final ElevatorSim elevatorSim = new ElevatorSim(
        DCMotor.getKrakenX60(2),
        4.0,
        8.0,
        Units.inchesToMeters(0.75),
        0.0,
        Units.inchesToMeters(55.0), 
        true,
        0.0);

    public Pose3d getStageComponentPose() {
        Transform3d transform3d = new Transform3d();
        if (this.getHeight() > Units.inchesToMeters(23.25))
            transform3d = new Transform3d(0.0, 0.0, this.getHeight() - Units.inchesToMeters(23.25), new Rotation3d());

        return new Pose3d(0.0, 0.0, 0.0, new Rotation3d()).plus(transform3d);
    }

    public Pose3d getCarriageComponentPose() {
        return new Pose3d(0.0, 0.0, this.getHeight(), new Rotation3d());
    }

    public void simulationUpdate() {
        if (!isZeroed) return;
        this.elevatorSim.setInputVoltage(this.main.get() * 12.0);
        this.elevatorSim.update(0.020);
        this.main.setPosition(this.elevatorSim.getPositionMeters());
        // this.main.setPosition(0.5 * Units.inchesToMeters(50) * (1 + Math.sin(2.0 * Math.PI / 5.0 * Timer.getTimestamp())));

        this.stageComponent.accept(this.getStageComponentPose());
        this.carriageComponent.accept(this.getCarriageComponentPose());
    }
}
