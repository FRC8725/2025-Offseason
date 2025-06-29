package frc.robot.lib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    public static Translation2d mirror(Translation2d translation2d) {
        return new Translation2d(Constants.Field.FIELD_X_SIZE - translation2d.getX(), translation2d.getY());
    }

    public static Rotation2d mirror(Rotation2d rotation2d) {
        return Rotation2d.kPi.minus(rotation2d);
    }

    public static Pose2d mirror(Pose2d pose) {
        return new Pose2d(mirror(pose.getTranslation()), mirror(pose.getRotation()));
    }

    public static double unclampedDeadzone(double inpt, double zone) {
        return Math.abs(inpt) < zone ? 0.0 : inpt;
    }
}
