package frc.robot.controllers;


import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/**
 * An abstract class for the joystick wrappers. Contains the common methods that we wrap around.
 */
public abstract class AbstractController<T extends CommandGenericHID> {
    public final T controller;

    protected AbstractController(T controller) {
        this.controller = controller;
    }

    public abstract double getRightHorizontalMovement();
    
    public abstract double getRightVerticalMovement();

    public abstract double getLeftHorizontalMovement();
    
    public abstract double getLeftVerticalMovement();

    public abstract boolean getRawButtonWrapper(int button);

    public abstract boolean getRawButtonReleasedWrapper(int button);

    public abstract boolean getRawButtonPressedWrapper(int button);
}
