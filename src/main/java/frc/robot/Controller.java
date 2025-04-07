package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller extends XboxController {
    public Controller() {
        super(0);
    }

    public Trigger Down() {
        return new Trigger(this::getLeftBumperButton);
    }
    public Trigger Up() {
        return new Trigger(this::getRightBumperButton);
    }
    public Trigger test(){
        return new Trigger(this::getXButton);
    }
    public Trigger start(){
        return new Trigger(this::getAButton);
    }
    public Trigger stop(){
        return new Trigger(this::getBButton);
    }
}