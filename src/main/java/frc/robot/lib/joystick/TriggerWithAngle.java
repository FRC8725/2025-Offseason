package frc.robot.lib.joystick;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerWithAngle extends Trigger {
    private final double elevatorAngle;
    private final double lifterAngle;

    public TriggerWithAngle(Trigger trigger, double elevatorAngle, double lifterAngle) {
        super(trigger);
        this.elevatorAngle = elevatorAngle;
        this.lifterAngle = lifterAngle;
    }
    
    public double getElevatorAngle() {
        return this.elevatorAngle;
    }

    public double getLifterAngle() {
        return this.lifterAngle;
    }
}