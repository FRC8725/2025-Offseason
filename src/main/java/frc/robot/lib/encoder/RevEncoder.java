package frc.robot.lib.encoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class RevEncoder extends DutyCycleEncoder {
    public RevEncoder(int port) {
        super(port);
    }

    // Max & Min is Degrees
    public int isInRange(double max, double min) {
        if (this.get() > max) return 1; // Above Range
        else if (this.get() <= max && this.get() >= min) return 0; // In Range
        else return -1; // Under Range
    }

    public double getRpmPosition() {
        return Units.rotationsToDegrees(this.get());
    }
}
