package frc.robot.subsystems;

import java.util.function.Supplier;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    // ---------- Object ---------- //
    private final TalonFX lifter = new TalonFX(0);
    private final TalonFX roller = new TalonFX(20);
    private final TalonFX center = new TalonFX(21);
    private final LaserCan laserCan = new LaserCan(0);
    private final Supplier<Boolean> wantGroundIntake;

    private boolean isZeroed = false;

    // ---------- State ---------- //
    private LifterState lifterState = LifterState.Up;
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
        In(-4.0, -8.0),
        SlowIn(-2.0, -3.0),
        TroughOut(3.25, 0.0),
        Out(8.0, -3.0),
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

    public Intake(Supplier<Boolean> wantGroundIntake) {
        this.configMotor();
        this.setLaserCanConfig();
        this.setZeroPosition();
        this.wantGroundIntake = wantGroundIntake;
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
        lifterConfig.Slot0 = slot0;

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive);

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
    public boolean hasCoral() {
        Measurement measurement = this.laserCan.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            if (measurement.distance_mm < Constants.Intake.LASER_DISTANCE) return true;
        }
        return false;
    }

    public LifterState getEffectiveLifterState() {
        if (lifterState != LifterState.OperatorControl) return lifterState;
        else if (this.hasCoral()) return LifterState.Up;
        else if (this.wantGroundIntake.get()) return LifterState.Down;
        else return LifterState.Up;
    }

    public RollerState getEffectiveRollerState() {
        if (rollerState != RollerState.OperatorControl) return rollerState;
        else if (this.wantGroundIntake.get()) return RollerState.In;
        else if (this.hasCoral()) return RollerState.SlowIn;
        else return RollerState.Off;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle", () -> Units.rotationsToDegrees(this.lifter.getPosition().getValueAsDouble()), null);
        builder.addBooleanProperty("IsZeroed", () -> this.isZeroed, null);
        builder.addStringProperty("State", () -> rollerState.toString(), null);
    }
}
