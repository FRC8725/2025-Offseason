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
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private static Elevator ELEVATOR;
    // ---------- Object ---------- //
    private final TalonFX main = new TalonFX(13);
    private final TalonFX follower = new TalonFX(14);
    private final Follower follower2 = new Follower(this.main.getDeviceID(), true);
    public final StatusSignal<Current> statorCurrent = this.main.getSupplyCurrent();
    private final MotionMagicVoltage request = new MotionMagicVoltage(0);
    private double lastClampedSetpointForLogging = 0.0;

    public static boolean isZeroed = false;

    // ---------- State ---------- //
    public static State state = State.Down;
    public enum State {
        Down(0.0),
        PreHandoff(Units.inchesToMeters(36.0)),
        Handoff(Units.inchesToMeters(35.25)),
        PopciclePickup(0.065),
        PreScore(Units.inchesToMeters(20.0)),
        Through(Units.inchesToMeters(38.0)),
        L2(Units.inchesToMeters(10.0)),
        L3(L2.value + Units.inchesToMeters(15.25)),
        L4(Units.inchesToMeters(49.0)),
        Barge(Units.inchesToMeters(49.0)),
        ScoreL2(L2.value + Units.inchesToMeters(5)),
        ScoreL3(L3.value - Units.inchesToMeters(3.5)),
        ScoreL4(L4.value - Units.inchesToMeters(1.0)),
        PostL2(L2.value - Units.inchesToMeters(3.5)), // TODO: Tune
        PostL3(L2.value - Units.inchesToMeters(6.0)), // TODO: Tune
        LowAglae(Units.inchesToMeters(22.25)),
        HighAglae(LowAglae.value + Units.inchesToMeters(15.8701)),
        AutoAlgae(Units.inchesToMeters(21.75)),
        AlgaeRest(Units.inchesToMeters(15.0)),
        SourceIntake(Units.inchesToMeters(53.0)),
        Processor(Units.inchesToMeters(20.0)),
        GroundAlgaeIntake(0.14);

        private final double value;

        State(double value) {
            this.value = value;    
        }
    }

    // ----------- Form ----------- //
    public static final InterpolatingDoubleTreeMap armToElevator = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap armToElevatorWhenIntakeDown = new InterpolatingDoubleTreeMap();

    public Elevator() {
        ELEVATOR = this;
        this.configMotor();
        this.setZeroPositon();

        for (Pair<Double, Double> pair : Constants.armElevatorPairs) {
            armToElevator.put(pair.getFirst(), pair.getSecond() + Units.inchesToMeters(0.5)) ;
        }
        for (Pair<Double, Double> pair : Constants.armInterpolationIntakeDown) {
            armToElevatorWhenIntakeDown.put(pair.getFirst(), pair.getSecond() + Units.inchesToMeters(0.5));
        }
    }

    public static Elevator getInstance() {
        return ELEVATOR;
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
        slot0Config.kG = 0.29;
        slot0Config.kP = 70.0;

        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig
            .withMotionMagicAcceleration(14.0)
            .withMotionMagicCruiseVelocity(3.0 / 5.0);

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

    public void setVoltage(double voltage) {
        this.main.setVoltage(voltage);
        this.follower.setControl(this.follower2);
    }

    public void stop() {
        this.main.setVoltage(0.0);
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
        this.statorCurrent.refresh();
        this.main.setControl(this.request.withPosition(this.clampSetpoint(state.value)));
        this.follower.setControl(this.follower2);
    }

    // ---------- Function ---------- //
    public boolean atSetpoint() {
        return Math.abs(this.main.getPosition().getValueAsDouble() - state.value) < Constants.Elevator.TOLERANCE;
    }

    public boolean lazierAtSetpoint() {
        return Math.abs(this.getHeight() - state.value) < Constants.Elevator.LAZIER_TOLERANCE;
    }

    public double getHeight() {
        return this.main.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return this.main.getVelocity().getValueAsDouble();
    }

    public double clampSetpoint(double v) {
        double ret = 0.0;
        double startTime = System.currentTimeMillis();

        double armDesiredPositionSignum = Math.signum(Arm.getInstance().getDesiredPosition());
        double angleForInterpolation;

        if (Math.signum(Arm.getInstance().getPosition()) != armDesiredPositionSignum) {
            angleForInterpolation = 0.0;
        } else if ((Arm.getInstance().getPosition() < 0.0 && Arm.getInstance().getDesiredPosition() > Arm.getInstance().getPosition()) ||
           (Arm.getInstance().getPosition() > 0.0 && Arm.getInstance().getDesiredPosition() < Arm.getInstance().getPosition())) {
            angleForInterpolation = Arm.getInstance().getDesiredPosition();
        } else {
            angleForInterpolation = Arm.getInstance().getPosition();
        }

        double interpolationTableInput = Math.PI - Math.abs(MathUtil.angleModulus(angleForInterpolation));

        double interpolatedValue = (Intake.getInstance().getEffectiveLifterState() == Intake.LifterState.Down && Intake.getInstance().atSetpoint()) ?
            armToElevatorWhenIntakeDown.get(interpolationTableInput) :
            armToElevator.get(interpolationTableInput);

        ret = Math.max(interpolatedValue, Math.min(v, Constants.Elevator.MAX_EXTENSION));
        this.lastClampedSetpointForLogging = ret;
        
        if (System.currentTimeMillis() - startTime > 5.0) System.out.println("Elevator.clampSetpoint() took " + startTime + " ms");
        return ret;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Voltage", () -> this.main.getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("Raw Velocity", () -> this.getVelocity(), null);
        builder.addDoubleProperty("Raw Acceleration", () -> this.main.getAcceleration().getValueAsDouble(), null);
        builder.addDoubleProperty("Height", () -> this.getHeight(), null);
        builder.addDoubleProperty("MotionMagic Setpoint", () -> this.main.getClosedLoopReference().getValueAsDouble(), null);
        builder.addDoubleProperty("Stator Current", () -> this.statorCurrent.getValueAsDouble(), null);
        builder.addBooleanProperty("atSetpoint", () -> this.atSetpoint(), null);
        builder.addBooleanProperty("isZeroed", () -> isZeroed, null);
        builder.addStringProperty("State", () -> state.toString(), null);
    }

    // ---------- Simulation ---------- //
    private final TalonFXSimState simState = this.main.getSimState();
    private final StructPublisher<Pose3d> stageComponent = NetworkTableInstance.getDefault()
        .getStructTopic("Component/StageComponent", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> carriageComponent = NetworkTableInstance.getDefault()
        .getStructTopic("Component/CarriageComponent", Pose3d.struct).publish();
    private final ElevatorSim elevatorSim = new ElevatorSim(
        DCMotor.getKrakenX60(2),
        4.0,
        5.0,
        Units.inchesToMeters(0.75),
        0.0,
        Constants.Elevator.MAX_EXTENSION, 
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
        this.elevatorSim.setInputVoltage(this.simState.getMotorVoltage());
        this.elevatorSim.update(0.02);

        double height = this.elevatorSim.getPositionMeters() * Constants.Elevator.MECHANISM_RATIO;
        double velocity = this.elevatorSim.getVelocityMetersPerSecond() * Constants.Elevator.MECHANISM_RATIO;

        this.simState.setRawRotorPosition(height);
        this.simState.setRotorVelocity(velocity);

        this.stageComponent.accept(this.getStageComponentPose());
        this.carriageComponent.accept(this.getCarriageComponentPose());
    }
}
