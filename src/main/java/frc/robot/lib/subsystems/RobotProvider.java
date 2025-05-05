package frc.robot.lib.subsystems;

import java.util.ArrayList;

public class RobotProvider {
    public static final ArrayList<SubsystemBase> subsystems = new ArrayList<>();

    public static void registerSubsystems(SubsystemBase subsystemBase) {
        subsystems.add(subsystemBase);
    }

    public static void periodic() {
        for (SubsystemBase subsystem : subsystems) {
            subsystem.periodic();
        }
    }
}
