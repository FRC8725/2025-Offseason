package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final TalonFX roller = new TalonFX(1);
    private final StatusSignal<Current> getCurrent = this.roller.getStatorCurrent();

    public Arm() {
        this.configRollerMotor(false);
        this.getCurrent.setUpdateFrequency(100.0);
    }

    public void configRollerMotor(boolean reverse) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(reverse ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
        config.CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(80.0);
        this.roller.getConfigurator().apply(config);
    }

    public enum RollerState {
        off(0.0),
        idle(-0.035),
        in(-1.0);

        public final double value;

        RollerState(double value) {
            this.value = value;
        }
    }

    RollerState rollerState = RollerState.off;
    Debouncer coralCurrentDebouncer = new Debouncer(0.15, DebounceType.kBoth);
    Debouncer algaeCurrentDebouncer = new Debouncer(0.35, DebounceType.kBoth);
    boolean hasObject = false;

    double ALGEA_UNDEBUNCED = 15.0;
    double CORAL_UNDEBUNCED = 10.0;

    @Override
    public void periodic() {
        this.getCurrent.refresh();
        boolean undebouncedHasObject = this.getCurrent.getValueAsDouble() > 15.0;

        boolean debouncedHasCoral = this.algaeCurrentDebouncer.calculate(undebouncedHasObject);
        this.hasObject = debouncedHasCoral;
        this.roller.set(rollerState.value);

        if (this.hasObject) rollerState = RollerState.idle;
        else rollerState = RollerState.in;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("HasObject", () -> this.hasObject, null);
        builder.addStringProperty("RollerState", () -> this.rollerState.toString(), null);
        builder.addDoubleProperty("RollerCurrent", () -> this.getCurrent.getValueAsDouble(), null);
    }
}
