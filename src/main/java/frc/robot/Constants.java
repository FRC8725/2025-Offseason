package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public final class Elevator {
        public static final double GEAR_RATIO = 4.0;
        public static final double SPOOL_RADIUS = Units.inchesToMeters(0.75);
        public static final double MECHANISM_RATIO = GEAR_RATIO / (2.0 * SPOOL_RADIUS * Math.PI);
        public static final double TOLERANCE = 0.01;
        public static final double ZERO_VOLTAGE = 0.3;
    }
}
