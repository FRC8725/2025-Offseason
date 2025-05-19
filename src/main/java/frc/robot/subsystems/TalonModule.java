package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonModule extends TalonFX {

    @SuppressWarnings("removal")
    public TalonModule(int port, boolean reverse, boolean isBrake) {
        super(port);
        this.clearStickyFaults();
        this.setInverted(reverse);
        this.setNeutralMode(isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        // this.resetSignalFrequencies();

        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(35.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(35.0);
        currentConfig.SupplyCurrentLimit = 120.0;
        currentConfig.SupplyCurrentLimitEnable = true;
        currentConfig.SupplyCurrentLowerLimit = 40.0;

        TalonFXConfiguration FXConfig = new TalonFXConfiguration().withCurrentLimits(currentConfig);
        this.getConfigurator().refresh(FXConfig);
        this.setPosition(0.0);
    }

    public void set(double speed) {
        super.set(speed);
    }

    public double getPositionValue() {
        return super.getPosition().getValueAsDouble();
    }

    public double getVelocityValue() {
        return super.getVelocity().getValueAsDouble();
    }
}