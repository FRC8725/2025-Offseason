package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    // ---------- Object ---------- //
    private final TalonFX lifter = new TalonFX(0);
    private final TalonFX roller = new TalonFX(20);
    private final TalonFX center = new TalonFX(21);

    private boolean isZeroed = false;

    // ---------- State ---------- //
    private LifterState lifterState = LifterState.Up;
    private RollerState rollerState = RollerState.In;
    public enum LifterState {
        Down(0.0),
        Through(0.0),
        Up(0.0);

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
        AlgaeModeIdle(0.0, 0.0);

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

    // ---------- Function ---------- //
    @Override
    public void periodic() {
        if (!this.isZeroed) return;
        // this.lifter.setControl(new MotionMagicVoltage(0.0));
        this.roller.setVoltage(this.rollerState.rollerVolt);
        this.center.setVoltage(this.rollerState.centerVolt);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle", () -> Units.rotationsToDegrees(this.lifter.getPosition().getValueAsDouble()), null);
        builder.addBooleanProperty("IsZeroed", () -> this.isZeroed, null);
    }
}
