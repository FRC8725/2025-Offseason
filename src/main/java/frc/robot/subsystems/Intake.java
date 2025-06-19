package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

    public static boolean hasCoral = false;
    private boolean isZeroed = false;

    // ---------- State ---------- //
    public static LifterState lifterState = LifterState.Up;
    public static RollerState rollerState = RollerState.In;
    public enum LifterState {
        Down(0.0),
        Through(0.0),
        Up(0.0),
        OperatorControl(0.0);

        public final double value;

        LifterState(double value) {
            this.value = value;
        }
    }

    public enum RollerState {
        In(-6.0, -8.0),
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

    public Intake() {
        this.configMotor();
        this.setZeroPosition();
    }

    // ---------- Config ---------- //
    public void configMotor() {
        TalonFXConfiguration lifterConfig = new TalonFXConfiguration();

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kS = 0.0;
        slot0.kV = 0.0;
        slot0.kA = 0.0;
        slot0.kG = 0.0;
        slot0.kP = 0.0;

        MotionMagicConfigs motionMagic = new MotionMagicConfigs();
        motionMagic.MotionMagicJerk = 2000.0;
        motionMagic.MotionMagicAcceleration = 2000.0;
        motionMagic.MotionMagicCruiseVelocity = 2.0;

        lifterConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast); // TODO: Need test
        lifterConfig.Feedback
            .withSensorToMechanismRatio(Constants.Intake.GEAR_RATIO);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive);

        this.lifter.getConfigurator().apply(lifterConfig);
        this.roller.getConfigurator().apply(rollerConfig);
        this.center.getConfigurator().apply(rollerConfig);
    }

    // ---------- Method ---------- //
    public void setZeroPosition() {
        this.lifter.setPosition(0.0);
        this.isZeroed = true;
    }

    public void setZeroVoltage() {
        this.lifter.setVoltage(Constants.Intake.ZERO_VOLTAGE);
    }

    public void stop() {
        this.lifter.setVoltage(0.0);
        this.roller.setVoltage(0.0);
        this.center.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        if (!this.isZeroed) return;
        // this.lifter.setControl(new MotionMagicVoltage(0.0));
        this.roller.setVoltage(rollerState.rollerVolt);
        this.center.setVoltage(rollerState.centerVolt);
    }

    // ---------- Function ---------- //
    public boolean atSetpoint() {
        return Math.abs(this.lifter.getPosition().getValueAsDouble() - lifterState.value) < Constants.Intake.TOLERANCE;
    }
    
    public double getPosition() {
        return Units.rotationsToRadians(this.lifter.getPosition().getValueAsDouble());
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
        4.0,
        0.35,
        0.0,
        Units.degreesToRadians(140.0),
        false,
        0.0);

    public Pose3d getIntakeComponentPose() {
        return new Pose3d(Units.inchesToMeters(11.25), 0.0, Units.inchesToMeters(7.5),
            new Rotation3d(0.0, this.getPosition(), 0.0));
    }

    public void simulationUpdate() {
        this.intakeSim.setInput(this.lifter.get());
        this.intakeSim.update(0.020);
        // this.lifter.setPosition(Units.degreesToRotations(0.5 * 140.0 * (1.0 + Math.sin(2.0 * Math.PI / 5.0 * Timer.getTimestamp()))));
        this.lifter.setPosition(Units.radiansToRotations(this.intakeSim.getAngleRads()));

        this.intakeComponent.accept(this.getIntakeComponentPose());
    }
}
