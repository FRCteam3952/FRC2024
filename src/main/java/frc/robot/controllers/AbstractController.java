package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An abstract class for the joystick wrappers. Contains the common methods that we wrap around.
 */
public abstract class AbstractController {
    public abstract double getRightHorizontalMovement();
    
    public abstract double getRightVerticalMovement();

    public abstract double getLeftHorizontalMovement();
    
    public abstract double getLeftVerticalMovement();

    public abstract boolean getRawButtonWrapper(int button);

    public abstract boolean getRawButtonReleasedWrapper(int button);

    public abstract boolean getRawButtonPressedWrapper(int button);

    /**
     * This method should not be used, and it is preferable to use one of the wrappers so that buttons are applicable to different controller implementations since WPILIB doesn't abstract much of this for us.
     * @param button The raw button number (check Driver Station)
     * @return A bindable {@link Trigger} for the requested button ID.
     */
    public abstract Trigger button(int button);

    /**
     * On the right hand side on controllers, there's four buttons arranged in a diamond shape. Due to what I can only assume to be a stylistic decision, the PS5 controller decided to not go with X/Y/A/B, hence the interesting name choice.
     * @return A bindable {@link Trigger} for the button at the top of the diamond on the right side of the controller.
     */
    public abstract Trigger upperButton();

    /**
     * On the right hand side on controllers, there's four buttons arranged in a diamond shape. Due to what I can only assume to be a stylistic decision, the PS5 controller decided to not go with X/Y/A/B, hence the interesting name choice.
     * @return A bindable {@link Trigger} for the button on the left of the diamond on the right side of the controller.
     */
    public abstract Trigger leftButton();

    /**
     * On the right hand side on controllers, there's four buttons arranged in a diamond shape. Due to what I can only assume to be a stylistic decision, the PS5 controller decided to not go with X/Y/A/B, hence the interesting name choice.
     * @return A bindable {@link Trigger} for the button on the right of the diamond on the right side of the controller.
     */
    public abstract Trigger rightButton();

    /**
     * On the right hand side on controllers, there's four buttons arranged in a diamond shape. Due to what I can only assume to be a stylistic decision, the PS5 controller decided to not go with X/Y/A/B, hence the interesting name choice.
     * @return A bindable {@link Trigger} for the button at the bottom of the diamond on the right side of the controller.
     */
    public abstract Trigger lowerButton();
}
