package frc.robot.lib.motors;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.SwerveConstants;

public class SwerveSpark extends SparkMax {
    private final double GEAR_RATIO;

    public SwerveSpark(int id, boolean reverse, boolean isDrive, double gearRatio) {
        super(id, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config 
            .inverted(reverse)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(isDrive ? 80 : 20)
            .voltageCompensation(isDrive ? SwerveConstants.MAX_DRIVE_VOLTAGE : SwerveConstants.MAX_TURN_VOLTAGE);
            // .closedLoopRampRate(0.1);
        this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.GEAR_RATIO = gearRatio;
    }

    public double getVelocity() {
        return this.getEncoder().getVelocity() / 60.0 * 2.0 * SwerveConstants.WHEEL_RADIUS * Math.PI / this.GEAR_RATIO;
    }

    public double getPosition() {
        return this.getEncoder().getPosition() * 2.0 * SwerveConstants.WHEEL_RADIUS * Math.PI / this.GEAR_RATIO;
    }
}
