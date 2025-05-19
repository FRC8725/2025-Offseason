package frc.robot.subsystems;

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
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final TalonFX roller = new TalonFX(1);
    private final StatusSignal<Current> statorCurrent = this.roller.getStatorCurrent();
    private boolean hasObject = false;

    // ----------- Debouncer ----------- //
    private final Debouncer coralCurrentDebouncer = new Debouncer(0.15, DebounceType.kBoth);
    private final Debouncer algaeCurrentDebouncer = new Debouncer(0.35, DebounceType.kBoth);

    // ----------- State ----------- //
    RollerState rollerState = RollerState.off;
    public enum RollerState {
        off(0.0),
        idle(-0.035),
        algeaIdle(-0.225),
        in(-1.0);

        public final double value;

        RollerState(double value) {
            this.value = value;
        }
    }

    public Arm() {
        this.configRollerMotor(false);
        this.statorCurrent.setUpdateFrequency(100.0);
    }

    // ----------- Config ----------- //
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

    @Override
    public void periodic() {
        this.statorCurrent.refresh();
        boolean undebouncedHasObject = this.statorCurrent.getValueAsDouble() >
            (this.rollerState == RollerState.idle ?
                Constants.Arm.IDEL_CURRENT_DRAW : Constants.Arm.CURRENT_DRAW);

        boolean debouncedHasCoral = this.coralCurrentDebouncer.calculate(undebouncedHasObject);
        boolean debouncedHasAlgae = this.algaeCurrentDebouncer.calculate(undebouncedHasObject);

        this.hasObject = (this.rollerState == RollerState.algeaIdle) ? debouncedHasAlgae : debouncedHasCoral;
        this.roller.set(rollerState.value);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("HasObject", () -> this.hasObject, null);
        builder.addStringProperty("RollerState", () -> this.rollerState.toString(), null);
        builder.addDoubleProperty("RollerCurrent", () -> this.statorCurrent.getValueAsDouble(), null);
    }
}
