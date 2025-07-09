package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.sensor.LaserCan;

public class Intake extends SubsystemBase {
    private static Intake INTAKE;
    // ---------- Object ---------- //
    private final TalonFX lifter = new TalonFX(17);
    private final TalonFX roller = new TalonFX(20);
    private final TalonFX center = new TalonFX(21);
    private final LaserCan laserCan = new LaserCan(50);

    public static boolean hasCoral = false;
    public static boolean isZeroed = false;

    // ---------- State ---------- //
    public static LifterState lifterState = LifterState.Up;
    public static RollerState rollerState = RollerState.Off;
    public enum LifterState {
        Down(Units.degreesToRadians(140.0)),
        Through(Units.degreesToRadians(37.0)),
        Up(0.0),
        OperatorControl(0.0);

        public final double value;

        LifterState(double value) {
            this.value = value;
        }
    }

    public enum RollerState {
        In(-6.0, -8.0),
        SlowIn(0.0, -6.0),
        TroughOut(4.0, -3.0),
        Out(8.0, 0.0),
        Off(0.0, 0.0),
        AlgaeModeIdle(0.0, 0.0),
        OperatorControl(0.0, 0.0);

        public final double rollerVolt;
        public final double centerVolt;

        RollerState(double rollerVolt, double centerVolt) {
            this.rollerVolt = rollerVolt;
            this.centerVolt = centerVolt;
        }
    }

    public Intake() {
        INTAKE = this;
        this.configMotor();
        this.setZeroPosition();
        this.simState.Orientation = ChassisReference.Clockwise_Positive;
    }

    public static Intake getInstance() {
        return INTAKE;
    }

    // ---------- Config ---------- //
    public void configMotor() {
        TalonFXConfiguration lifterConfig = new TalonFXConfiguration();

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kS = 0.0;
        slot0.kV = 0.0;
        slot0.kA = 0.0;
        slot0.kG = 0.0;
        slot0.kP = 160.0;

        lifterConfig.MotionMagic
            .withMotionMagicJerk(2000.0)
            .withMotionMagicAcceleration(200.0)
            .withMotionMagicCruiseVelocity(2.0 / 2.0);

        lifterConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        lifterConfig.Feedback
            .withSensorToMechanismRatio(Constants.Intake.GEAR_RATIO);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive);
        lifterConfig.Slot0 = slot0;

        this.lifter.getConfigurator().apply(lifterConfig);
        this.roller.getConfigurator().apply(rollerConfig);
        this.center.getConfigurator().apply(rollerConfig);
    }

    // ---------- Method ---------- //
    public void setZeroPosition() {
        this.lifter.setPosition(0.0);
        isZeroed = true;
    }

    public void setZeroVoltage() {
        this.lifter.setVoltage(Constants.Intake.ZERO_VOLTAGE);
    }

    public static void setState(LifterState lifter, RollerState roller) {
        lifterState = lifter;
        rollerState = roller;
    }

    public void stop() {
        this.lifter.setVoltage(0.0);
        this.roller.setVoltage(0.0);
        this.center.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        if (!isZeroed) return;
        if (RobotBase.isReal()) hasCoral = this.hasCoral();
        this.lifter.setControl(new MotionMagicVoltage(Units.radiansToRotations(this.getEffectiveLifterState().value)));
        this.roller.setVoltage(this.getEffectiveRollerState().rollerVolt);
        this.center.setVoltage(this.getEffectiveRollerState().centerVolt);
    }

    // ---------- Function ---------- //
    public boolean atSetpoint() {
        return Math.abs(this.getPosition() - this.getEffectiveLifterState().value) < Constants.Intake.TOLERANCE;
    }
    
    public double getPosition() {
        return Units.rotationsToRadians(this.lifter.getPosition().getValueAsDouble());
    }

    public double getVelocity() {
        return Units.rotationsToRadians(this.lifter.getVelocity().getValueAsDouble());
    }

    public boolean isUnsafeToGoUp() {
        return Math.abs(MathUtil.angleModulus(Arm.getInstance().getPosition())) < Math.PI - Arm.elevatorToArm.get(Elevator.getInstance().getHeight());
    }

    public boolean hasCoral() {
        Measurement measurement = this.laserCan.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            if (measurement.distance_mm < Constants.Intake.LASERCAN_DISTANCE) return true;
        }
        return false;
    }

    public LifterState getEffectiveLifterState() {
        if (lifterState != LifterState.OperatorControl) return lifterState;
        else if (this.isUnsafeToGoUp()) return LifterState.Down;
        else if (this.hasCoral()) return LifterState.Up;
        else if (SuperStructure.getInstance().input.wantGroundIntake) return LifterState.Down;
        else return LifterState.Up;
    }

    public RollerState getEffectiveRollerState() {
        if (rollerState != RollerState.OperatorControl) return rollerState;
        else if (SuperStructure.getInstance().input.wantGroundIntake) return RollerState.In;
        else if (this.hasCoral()) return RollerState.SlowIn;
        else return RollerState.Off;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle", () -> Units.rotationsToDegrees(this.lifter.getPosition().getValueAsDouble()), null);
        builder.addDoubleProperty("Voltage", () -> this.lifter.getMotorVoltage().getValueAsDouble(), null);
        builder.addBooleanProperty("IsZeroed", () -> isZeroed, null);
        builder.addBooleanProperty("HasCoral", () -> hasCoral, null);
        builder.addBooleanProperty("atsetpoint", () -> this.atSetpoint(), null);
        builder.addBooleanProperty("isUnsafe", () -> this.isUnsafeToGoUp(), null);
        builder.addStringProperty("RollerState", () -> rollerState.toString(), null);
        builder.addStringProperty("Effective RollerState", () -> this.getEffectiveRollerState().toString(), null);
        builder.addStringProperty("LifterState", () -> this.getEffectiveLifterState().toString(), null);
    }

    // --------- Simulation ---------- //
    private final TalonFXSimState simState = this.lifter.getSimState();
    private final StructPublisher<Pose3d> intakeComponent = NetworkTableInstance.getDefault()
        .getStructTopic("Component/Intake",  Pose3d.struct).publish();
    private final SingleJointedArmSim intakeSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(1),
        Constants.Intake.GEAR_RATIO,
        SingleJointedArmSim.estimateMOI(0.25, 3.5),
        0.25,
        0.0,
        Units.degreesToRadians(140.0),
        false,
        0.0);

    public Pose3d getIntakeComponentPose() {
        return new Pose3d(Units.inchesToMeters(11.25), 0.0, Units.inchesToMeters(7.5),
            new Rotation3d(0.0, this.getPosition(), 0.0));
    }

    public void simulationUpdate() {
        this.intakeSim.setInputVoltage(this.simState.getMotorVoltage());
        this.intakeSim.update(0.020);

        double angleRad = this.intakeSim.getAngleRads();
        double velocity = this.intakeSim.getVelocityRadPerSec();

        double angleRadTicks = angleRad * Constants.Intake.GEAR_RATIO / (2.0 * Math.PI);
        double velocityTicks = velocity * Constants.Intake.GEAR_RATIO / (2.0 * Math.PI);

        this.simState.setRawRotorPosition(angleRadTicks);
        this.simState.setRotorVelocity(velocityTicks);

        this.intakeComponent.accept(this.getIntakeComponentPose());
    }
}
