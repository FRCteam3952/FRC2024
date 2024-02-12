package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A wrapper around {@link CommandXboxController}. Our left joystick is not working right now though, so I'm just going to use the right side one for now.
 */
public class XboxController extends AbstractController{
    public static final double IGNORE_DELTA = 0.08;

    private final CommandXboxController controller;
    public XboxController(CommandXboxController controller) {
        this.controller = controller;
    }

    /**
     * The XBox controller sometimes returns a value of 0.01 to 0.03 when at rest. To avoid floating point errors, this method corrects for that.
     *
     * @param value The value to correct
     * @return 0.0 if the value is within the deadzone {@link #IGNORE_DELTA}, otherwise the value
     */
    private static double correctDeadzone(double value) {
        if (Math.abs(value) < IGNORE_DELTA) {
            return 0;
        } else {
            return value;
        }
    }

    @Override
    public double getRightHorizontalMovement() {
        return correctDeadzone(controller.getRightX());
    }

    @Override
    public double getRightVerticalMovement() {
        return correctDeadzone(controller.getRightY());
    }

    @Override
    public double getLeftHorizontalMovement() {
        return correctDeadzone(controller.getLeftX());
    }

    @Override
    public double getLeftVerticalMovement() {
        return correctDeadzone(controller.getLeftY());
    }

    @Override
    public boolean getRawButtonWrapper(int button) {
        return controller.getHID().getRawButton(button);
    }

    @Override
    public boolean getRawButtonReleasedWrapper(int button) {
        return controller.getHID().getRawButtonReleased(button);
    }

    @Override
    public boolean getRawButtonPressedWrapper(int button) {
        return controller.getHID().getRawButtonPressed(button);
    }

    @Override
    public Trigger button(int button) {
        return this.controller.button(button);
    }

    @Override
    public Trigger upperButton() {
        return this.controller.x();
    }

    @Override
    public Trigger leftButton() {
        return this.controller.y();
    }

    @Override
    public Trigger rightButton() {
        return this.controller.a();
    }

    @Override
    public Trigger lowerButton() {
        return this.controller.b();
    }

    @Override
    public int getPOV() {
        return this.controller.getHID().getPOV();
    }

    @Override
    public Trigger leftShoulderButton() {
        return this.controller.leftBumper();
    }

    @Override
    public Trigger rightShoulderButton() {
        return this.controller.rightBumper();
    }

    @Override
    public Trigger leftShoulderTrigger() {
        return this.controller.leftTrigger();
    }

    @Override
    public Trigger rightShoulderTrigger() {
        return this.controller.rightTrigger();
    }
}
