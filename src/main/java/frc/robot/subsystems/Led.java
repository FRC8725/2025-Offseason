package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.helpers.LedHelper;
import frc.robot.subsystems.SuperStructure.State;

public class Led extends SubsystemBase {
    private final int LED_PORT = 0;
    private final int LED_BUFFER_LENGTH = 19; // TODO
    private boolean idleMode = false;

    private final AddressableLED led = new AddressableLED(LED_PORT);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_BUFFER_LENGTH);

    public Led() {
        this.led.setLength(LED_BUFFER_LENGTH);
        this.led.setData(this.buffer);
        this.led.start();
    }

    public void setIdleMode(boolean mode) {
        this.idleMode = mode;
    }

    public void defaultMode() {
        // TODO
        Map<Double, Color> maskSteps = Map.of(0.0, LedHelper.White.color, 0.2, LedHelper.Black.color);
        LEDPattern base = LEDPattern.solid(LedHelper.Orange.color);
        LEDPattern mask =
        LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(75.0));
        LEDPattern pattern = base.mask(mask);
        // LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, LedHelper.Orange.color);
        // LEDPattern pattern = base.blink(Seconds.of(0.1));
        pattern.applyTo(this.buffer);
    }

    public void flashRed() {
        LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, LedHelper.Red.color);
        LEDPattern pattern = base.blink(Seconds.of(0.1));
        pattern.applyTo(this.buffer);
    }

    public void flashGreen() {
        LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, LedHelper.Green.color);
        LEDPattern pattern = base.blink(Seconds.of(0.08));
        pattern.applyTo(this.buffer);
    }

    public void intakeHasCoral() {
        LEDPattern pattern = LEDPattern.solid(LedHelper.DarkBlue.color);
        pattern.applyTo(this.buffer);
    }

    public void armHasCoral() {
        LEDPattern pattern = LEDPattern.solid(LedHelper.Aque.color);
        pattern.applyTo(this.buffer);
    }

    // private void rainbow() {
    //     LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    //     Distance spacing = Meters.of(1.0 / 120.0);
    //     LEDPattern pattern = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), spacing);
    //     pattern.applyTo(this.buffer);
    // }

    @Override
    public void periodic() {
        if (this.idleMode) {
            this.defaultMode();
        } else if (Swerve.getInstance().isAligned) {
            this.flashGreen();
        } else if (SuperStructure.getInstance().state == State.PlaceL2 ||
                SuperStructure.getInstance().state == State.PlaceL3 ||
                SuperStructure.getInstance().state == State.PlaceL4) {
            this.flashRed();
        } else if (Intake.hasCoral) {
            this.intakeHasCoral();
        } else if (Arm.hasObject) {
            this.armHasCoral();
        } else {
            this.defaultMode();
        }
        this.led.setData(this.buffer);
    }
}
