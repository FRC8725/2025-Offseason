package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
    private final int LED_PORT = 0; // TODO
    private final int LED_BUFFER_LENGTH = 0; // TODo
    private boolean idleMode = true;

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
    }

    public void flashRed() {
        LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, Color.kRed);
        LEDPattern pattern = base.blink(Seconds.of(0.1));
        pattern.applyTo(this.buffer);
    }

    public void flashGreen() {
        LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, Color.kGreen);
        LEDPattern pattern = base.blink(Seconds.of(0.1));
        pattern.applyTo(this.buffer);
    }

    @Override
    public void periodic() {
        if (this.idleMode) this.defaultMode();
        this.led.setData(this.buffer);
    }
}
