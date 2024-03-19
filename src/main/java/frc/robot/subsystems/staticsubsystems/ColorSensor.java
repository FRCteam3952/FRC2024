package frc.robot.subsystems.staticsubsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public final class ColorSensor {
    private static final Color NOTE_COLOR = new Color(132, 96, 27);

    private static final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    private static final ColorMatch colorMatcher = new ColorMatch();

    public static void initialize() {
        colorMatcher.addColorMatch(NOTE_COLOR);
    }

    public static Color getColor() {
        return colorSensor.getColor();
    }

    /**
     * Returns whether the color readout matches the target note color.
     *
     * @return Whether the color readout matches the target note color.
     */
    public static boolean isNoteColor() {
        ColorMatchResult matchResult = colorMatcher.matchColor(getColor());
        if (matchResult == null) {
            return false;
        }
        System.out.println("match result: " + colorAsRGBString(matchResult.color));
        return matchResult.color == NOTE_COLOR;
    }

    public static String colorAsRGBString(Color c) {
        return "Color[" + (int) (c.red * 256) + ", " + (int) (c.green * 256) + ", " + (int) (c.blue * 256) + "]";
    }
}
