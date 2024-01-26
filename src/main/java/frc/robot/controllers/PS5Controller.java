package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

/**
 * A wrapper around {@link CommandPS5Controller}. Our left joystick is not working right now though, so I'm just going to use the right side one for now.
 */
public class PS5Controller extends AbstractController {
    public static final double IGNORE_DELTA = 0.08;

    public final CommandPS5Controller controller;

    public PS5Controller(CommandPS5Controller controller) {
        this.controller = controller;
    }

    @Override
    public double getRightHorizontalMovement() {
        return controller.getRightX();
    }

    @Override
    public double getRightVerticalMovement() {
        return controller.getRightY();
    }

    @Override
    public double getLeftHorizontalMovement() {
        return controller.getLeftX();
    }

    @Override
    public double getLeftVerticalMovement() {
        return controller.getLeftY();
    }

    @Override
    public boolean getRawButtonWrapper(int button) {
        return controller.getHID().getRawButton(button);
    }

    @Override
    public boolean getRawButtonReleasedWrapper(int button) {
        return controller.getHID().getRawButtonReleased(button);
    }

    public boolean getRawButtonPressedWrapper(int button) {
        return controller.getHID().getRawButtonPressed(button);
    }
}
