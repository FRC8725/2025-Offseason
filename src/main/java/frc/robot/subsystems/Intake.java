package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    // ---------- Object ---------- //
    private final TalonFX lifter = new TalonFX(17);
    private final TalonFX roller = new TalonFX(20);
    private final TalonFX center = new TalonFX(21);
    private final LaserCan laserCan = new LaserCan(0);
    private final Supplier<SuperStructure.StructureInput> input;
    private final Supplier<Double> armPosition;
    private final Supplier<Double> elevatorHeight;

    public static boolean hasCoral = false;
    private boolean isZeroed = false;

    // ---------- State ---------- //
    public static LifterState lifterState = LifterState.Up;
    public static RollerState rollerState = RollerState.Off;
    public enum LifterState {
        Down(Units.degreesToRadians(126.0)),
        Through(Units.degreesToRadians(25.639507)),
        Up(0.0),
        OperatorControl(0.0);

        public final double value;

        LifterState(double value) {
            this.value = value;
        }
    }

    public enum RollerState {
        In(-6.0, -8.0),
        SlowIn(-2.0, -3.0),
        TroughOut(3.25, 0.0),
        Out(8.0, -4.0),
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

    public Intake(Supplier<SuperStructure.StructureInput> input, Supplier<Double> armPosition, Supplier<Double> elevatorHeight) {
        this.configMotor();
        this.setZeroPosition();
        this.input = input;
        this.armPosition = armPosition;
        this.elevatorHeight = elevatorHeight;
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
            .withMotionMagicCruiseVelocity(2.0);

        lifterConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast); // TODO: Need test
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

    public void setLaserCanConfig() {
        try {
            this.laserCan.setRangingMode(RangingMode.SHORT);
            this.laserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 16, 16));
            this.laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();
        }
    }

    // ---------- Method ---------- //
    public void setZeroPosition() {
        this.lifter.setPosition(0.0);
        this.isZeroed = true;
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
        if (!this.isZeroed) return;
        this.lifter.setControl(new MotionMagicVoltage(Units.radiansToRotations(this.getEffectiveLifterState().value)));
        this.roller.setVoltage(this.getEffectiveRollerState().rollerVolt);
        this.center.setVoltage(this.getEffectiveRollerState().centerVolt);
    }

    // ---------- Function ---------- //
    public boolean atSetpoint() {
        return Math.abs(this.lifter.getPosition().getValueAsDouble() - lifterState.value) < Constants.Intake.TOLERANCE;
    }
    
    public double getPosition() {
        return Units.rotationsToRadians(this.lifter.getPosition().getValueAsDouble());
    }

    public boolean isUnsafeToGoUp() {
        return Math.abs(MathUtil.angleModulus(this.armPosition.get())) < Math.PI - Arm.elevatorToArm.get(this.elevatorHeight.get());
    }

    public boolean hasCoral() {
        Measurement measurement = this.laserCan.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            if (measurement.distance_mm < Constants.Intake.LASER_DISTANCE) return true;
        }
        return false;
    }

    public LifterState getEffectiveLifterState() {
        if (lifterState != LifterState.OperatorControl) return lifterState;
        else if (this.isUnsafeToGoUp()) return LifterState.Down;
        else if (this.hasCoral()) return LifterState.Up;
        else if (this.input.get().wantGroundIntake) return LifterState.Down;
        else return LifterState.Up;
    }

    public RollerState getEffectiveRollerState() {
        if (rollerState != RollerState.OperatorControl) return rollerState;
        else if (this.input.get().wantGroundIntake) return RollerState.In;
        else if (this.hasCoral()) return RollerState.SlowIn;
        else return RollerState.Off;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle", () -> Units.rotationsToDegrees(this.lifter.getPosition().getValueAsDouble()), null);
        builder.addBooleanProperty("IsZeroed", () -> this.isZeroed, null);
        builder.addStringProperty("RollerState", () -> rollerState.toString(), null);
        builder.addStringProperty("LifterState", () -> lifterState.toString(), null);
    }

    // --------- Simulation ---------- //
    private final StructPublisher<Pose3d> intakeComponent = NetworkTableInstance.getDefault()
        .getStructTopic("Component/Intake",  Pose3d.struct).publish();
    private final SingleJointedArmSim intakeSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(1),
        160.0 / 3.0,
        5.0,
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
        this.intakeSim.setInputVoltage(this.lifter.getMotorVoltage().getValueAsDouble());
        this.intakeSim.update(0.020);
        // this.lifter.setPosition(Units.degreesToRotations(0.5 * 140.0 * (1.0 + Math.sin(2.0 * Math.PI / 5.0 * Timer.getTimestamp()))));
        this.lifter.setPosition(Units.radiansToRotations(this.intakeSim.getAngleRads()));

        this.intakeComponent.accept(this.getIntakeComponentPose());
    }
}
