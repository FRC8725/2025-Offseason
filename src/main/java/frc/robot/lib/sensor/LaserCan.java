package frc.robot.lib.sensor;

import au.grapplerobotics.ConfigurationFailedException;

public class LaserCan extends au.grapplerobotics.LaserCan {
    public LaserCan(int port) {
        super(port);

        try {
            this.setRangingMode(RangingMode.SHORT);
            this.setRegionOfInterest(new RegionOfInterest(8, 8, 16, 16));
            this.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();
        }
    }
}
