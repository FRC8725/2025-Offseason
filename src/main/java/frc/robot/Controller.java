package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller extends XboxController {
    public Controller() {
        super(0);
    }

    public Trigger Up() {
        return new Trigger(this::getRightBumperButton);
    }

    public Trigger Down() {
        return new Trigger(this::getLeftBumperButton);
    }

    public Trigger Start() {
        return new Trigger(this::getAButton);
    }

    public Trigger Stop() {
        return new Trigger(this::getXButton);
    }

    public Trigger Test() {
        return new Trigger(this::getYButton);
    }
}