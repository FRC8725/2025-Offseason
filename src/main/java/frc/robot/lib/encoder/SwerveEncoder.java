package frc.robot.lib.encoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveEncoder extends CANcoder {
    public SwerveEncoder(int id) {
        super(id);
        CANcoderConfiguration config = new CANcoderConfiguration();
        // config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        // config.MagnetSensor.MagnetOffset = 0.0;
        this.getConfigurator().apply(config);
    }

    public Rotation2d getRotation() {
        double value = this.getAbsolutePosition().getValueAsDouble();
        return Rotation2d.fromRotations(value > 0.5 ? value - 1.0 : value);
    }
}
