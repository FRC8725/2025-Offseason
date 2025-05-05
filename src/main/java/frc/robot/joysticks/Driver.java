package frc.robot.joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SwerveConstants;

public class Driver extends XboxController {
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(SwerveConstants.MAX_ACCELERATION);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(SwerveConstants.MAX_ACCELERATION);
    private final SlewRateLimiter rLimiter = new SlewRateLimiter(SwerveConstants.MAX_ANGULAR_ACCELERATION);

    public Driver() {
        super(0);
    }

    public double getXDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftY(), SwerveConstants.DEAD_BAND) * SwerveConstants.MAX_SPEED * this.getBrake();
        return this.xLimiter.calculate(speed);
    }

    public double getYDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftX(), SwerveConstants.DEAD_BAND) * SwerveConstants.MAX_SPEED * this.getBrake();
        return this.yLimiter.calculate(speed);
    }

    public double getRDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getRightX(), SwerveConstants.DEAD_BAND) * SwerveConstants.MAX_SPEED * this.getBrake();
        return this.rLimiter.calculate(speed);
    }

    public double getBrake() {
        double speed = MathUtil.applyDeadband(this.getRightTriggerAxis(), SwerveConstants.DEAD_BAND);
        return 1.0 - speed;
    }

    public Trigger resetGyro() {
        return new Trigger(this::getAButton);
    }

    public Trigger resetPosition() {
        return new Trigger(this::getYButton);
    }
}
