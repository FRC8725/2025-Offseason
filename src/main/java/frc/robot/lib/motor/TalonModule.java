package frc.robot.lib.motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonModule extends TalonFX {

    public TalonModule(int port, boolean reverse, boolean isBrake) {
        super(port);
        this.clearStickyFaults();
        this.resetSignalFrequencies(0.01);

        MotorOutputConfigs outputConfig = new MotorOutputConfigs();
        outputConfig.Inverted = reverse ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        outputConfig.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
        currentConfig.SupplyCurrentLimit = 30.0;
        currentConfig.SupplyCurrentLimitEnable = true;
        currentConfig.StatorCurrentLimit = 80.0;
        currentConfig.StatorCurrentLimitEnable = true;

        Slot0Configs slot0Config = new Slot0Configs();
        slot0Config.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Config.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Config.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Config.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Config.kI = 0; // no output for integrated error
        slot0Config.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = 80.0;
        motionMagicConfig.MotionMagicAcceleration = 160.0;
        motionMagicConfig.MotionMagicJerk = 1600.0; // Target jerk of 1600 rps/s/s (0.1 seconds)

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withCurrentLimits(currentConfig)
            .withMotorOutput(outputConfig)
            .withMotionMagic(motionMagicConfig)
            .withSlot0(slot0Config);
        this.getConfigurator().refresh(motorConfig);
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
