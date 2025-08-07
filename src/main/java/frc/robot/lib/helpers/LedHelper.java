package frc.robot.lib.helpers;

import edu.wpi.first.wpilibj.util.Color;

public enum LedHelper {
    White(new Color(255, 255, 255)),
    Red(new Color(255, 0, 0)),
    Green(new Color(0, 0, 255)),
    Black(new Color(0, 0, 0)),
    Orange(new Color(255, 7, 145)),
    Blue(new Color(79, 240, 90)),
    DarkBlue(new Color(13, 255, 53)),
    Aque(new Color(81, 247, 252));

    public final Color color;

    LedHelper(Color color) {
        this.color = color;
    }
}
