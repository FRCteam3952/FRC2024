package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A wrapper around {@link CommandXboxController} but for the Nintendo Pro Controller.
 * <p>
 * This class is necessary because the Nintendo Pro Controller has different deadzones than the XBox controller, as well as not reaching all values from [-1, 1] on joysticks.
 */
public class NintendoProController extends AbstractController {
    public static final double IGNORE_DELTA = 0.15;

    private final CommandXboxController controller;
    public NintendoProController(CommandXboxController controller) {
        this.controller = controller;
    }

    /**
     * Applies a deadzone and scale to the raw controller values. Necessary because the Pro Controller has some bounds issues where extremities do not return values equal to 1.
     * @param rawVal The raw value from the controller
     * @param maxValPos The maximum (farthest from 0) value possible for rawVal > 0. This value should be POSITIVE.
     * @param maxValNeg The maximum (farthest from 0) value possible for rawVal < 0. This value should be NEGATIVE.
     * @return A value, deadzoned to 0 if within {@link NintendoProController#IGNORE_DELTA IGNORE_DELTA}. If value is not deadzoned, positive values will be scaled such that maxValPos will become a value of 1, and negative values scaled similarly with maxValNeg
     */
    private static double correctDeadzone(double rawVal, double maxValPos, double maxValNeg) {
        double absVal = Math.abs(rawVal);
        if(absVal < IGNORE_DELTA) {
            return 0;
        }
        if(rawVal > 0) {
            return (absVal - IGNORE_DELTA) / (maxValPos - 0.15);
        }
        return (absVal - IGNORE_DELTA) / (maxValNeg + 0.15);
    }

    private double getControllerLeftX() {
        return correctDeadzone(controller.getRawAxis(0), 0.67, -0.89);
    }

    private double getControllerLeftY() {
        // NOTE: INVERTED. KEEP INVERTED
        return -correctDeadzone(-controller.getRawAxis(1), 0.74, -0.84);
    }

    private double getControllerRightX() {
        return correctDeadzone(controller.getRawAxis(2), 0.74, -0.76);
    }

    private double getControllerRightY() {
        // NOTE: INVERTED. KEEP INVERTED
        return -correctDeadzone(-controller.getRawAxis(3), 0.8, -0.76);
    }

    @Override
    public double getRightHorizontalMovement() {
        return this.getControllerRightX();
    }

    @Override
    public double getRightVerticalMovement() {
        return this.getControllerRightY();
    }

    @Override
    public double getLeftHorizontalMovement() {
        return this.getControllerLeftX();
    }

    @Override
    public double getLeftVerticalMovement() {
        return this.getControllerLeftY();
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
        return this.button(4);
    }

    @Override
    public Trigger leftButton() {
        return this.button(3);
    }

    @Override
    public Trigger rightButton() {
        return this.button(2);
    }

    @Override
    public Trigger lowerButton() {
        return this.button(1);
    }
}
