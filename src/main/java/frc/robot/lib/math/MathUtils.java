package frc.robot.lib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Robot;

public class MathUtils {
    public static double wrapTo0_2PI(double a) {
        double value = MathUtil.angleModulus(a);
        return value < 0.0 ? value + 2.0 * Math.PI : value;
    }

    public static Translation2d mirrorIfRed(Translation2d t) {
        return Robot.isRedAlliance ?
            new Translation2d(Constants.Field.FIELD_X_SIZE - t.getX(), t.getY()) : t;
    }
}
