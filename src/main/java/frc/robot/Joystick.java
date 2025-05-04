package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;

public class Joystick extends XboxController {
    private final SlewRateLimiter limiter = new SlewRateLimiter(0.5);

    public Joystick() {
        super(0);
    }

    public double getElevatorSpeed() {
        double speed = MathUtil.applyDeadband(this.getLeftY(), ElevatorConstans.DEAD_BAND);
        return this.limiter.calculate(speed);
    }
}
