package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.AbstractController;

public final class ControlHandler {
    public enum TriggerType {
        UPPER_BUTTON,
        LEFT_BUTTON,
        RIGHT_BUTTON,
        LOWER_BUTTON,
        LEFT_SHOULDER_BUTTON,
        RIGHT_SHOULDER_BUTTON,
        LEFT_SHOULDER_TRIGGER,
        RIGHT_SHOULDER_TRIGGER
    }
    public static Trigger get(AbstractController controller, TriggerType type) {
        return switch (type) {
            case UPPER_BUTTON -> controller.upperButton();
            case LEFT_BUTTON -> controller.leftButton();
            case RIGHT_BUTTON -> controller.rightButton();
            case LOWER_BUTTON -> controller.lowerButton();
            case LEFT_SHOULDER_BUTTON -> controller.leftShoulderButton();
            case RIGHT_SHOULDER_BUTTON -> controller.rightShoulderButton();
            case LEFT_SHOULDER_TRIGGER -> controller.leftShoulderTrigger();
            case RIGHT_SHOULDER_TRIGGER -> controller.rightShoulderTrigger();
        };
    }
}
