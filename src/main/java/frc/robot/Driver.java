package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.constants.SwerveConstants;

public class Driver extends XboxController {
    public Driver() {
        super(0);
    }

    public ChassisSpeeds getSpeeds() {
        double x = this.getLeftX();
        double y = this.getLeftY();
        double rotation = this.deadZone(this.getRightX(), 0.05);

        double theta = Math.atan2(y, x);
        double r = Math.hypot(x, y);

        r = this.deadZone(r, 0.05);
        r = r * r;
        rotation = rotation * rotation * Math.signum(rotation);

        double outputX = r * Math.sin(theta) * SwerveConstants.MAX_VELOCITY;
        double outputY = r * Math.cos(theta) * SwerveConstants.MAX_VELOCITY;
        double outputR = rotation * SwerveConstants.MAX_ANGULAR_VELOCITY;

        return new ChassisSpeeds(outputX, outputY, outputR);
    }

    private double deadZone(double value, double zone) {
        if (Math.abs(value) < zone) return 0.0;
        else if (value > 1.0) return 1.0;
        else if (value < -1.0) return -1.0;
        else return (value - zone) / (1.0 - zone);
    }
}
