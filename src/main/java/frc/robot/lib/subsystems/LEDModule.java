package frc.robot.lib.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDModule {
    private final int startIndex;
    private final int ledBufferLength;
    private final boolean reversed;
    private final AddressableLEDBuffer ledBuffer;

    private int rainbowFirstPixelHue = 0;
    private int floatStart = 0;

    public LEDModule(int startIndex, int lLedBufferLength, boolean reversed, AddressableLEDBuffer led) {
        this.startIndex = startIndex;
        this.ledBufferLength = lLedBufferLength;
        this.reversed = reversed;
        this.ledBuffer = led;
    }

    public void setAllColor(Color color) {
        for (int i = 0; i < this.ledBufferLength; i++) {
            this.ledBuffer.setLED(i + this.startIndex, color);
        }
    }

    public void setAllColorWithHSV(int h, int s, int v) {
        for (int i = 0; i < this.ledBufferLength; i++) {
            this.ledBuffer.setHSV(i, h, s, v);
        }
    }

    public void rainbow(int step) {
		for (int i = 0; i < this.ledBufferLength; i++) {
			final int hue = (this.rainbowFirstPixelHue + (i * 180 / this.ledBufferLength)) % 180;
			this.ledBuffer.setHSV(i, hue, 255, 128);
            this.ledBuffer.setHSV(this.ledBufferLength - (i + 1), hue, 255, 128);
        }

		this.rainbowFirstPixelHue += step;
		this.rainbowFirstPixelHue %= 180;
	}

    public void setFloatColor(Color color, int range, int step) {
        this.setAllColor(Color.kBlack);
        for (int i = 0; i < range; i++) {
            this.ledBuffer.setLED((this.floatStart + i), color);
        }

        this.floatStart += step;
        this.floatStart %= this.ledBufferLength;
    }

    public void setFloatColor(Color[] colors, int step) {
        this.setAllColor(Color.kBlack);
        for (int i = 0; i < colors.length; i++) {
            this.ledBuffer.setLED((this.floatStart + i), colors[i]);
        }

        this.floatStart += step;
        this.floatStart %= this.ledBufferLength;
    }

    public void setColorInRange(Color backgroundColor, Color color, int start, int end, int step) {
        if (start < this.startIndex) start = this.startIndex;
        if (end > this.startIndex + this.ledBufferLength) end = this.startIndex + this.ledBufferLength;
        if (start > end) return;

        for (int i = 0; i < this.ledBufferLength; i += step) {
            if (i >= start && i < end) this.ledBuffer.setLED(i + this.startIndex, this.reversed ? backgroundColor : color);
            else this.ledBuffer.setLED(i + this.startIndex, this.reversed ? color : backgroundColor);
        }
    }

    public void setColorWithPercentage(Color backgroundColor, Color color, double percentage) {
        percentage = this.reversed ? 1 - percentage : percentage;
        int end = (int) (this.ledBufferLength * percentage);
        this.setColorInRange(backgroundColor, color, 0, end, 1);
    }
}
