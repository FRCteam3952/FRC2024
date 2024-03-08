package frc.robot.controllers;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerHandler {
    public AbstractController controller;

    public ControllerHandler(AbstractController controller) {
        this.controller = controller;
    }

    public boolean getRunIntakeButton() {
        return this.controller.getUpperButton(); // idk
    }

    // public class Buttons extends Enumerable {
    //     TEST(controller::upperButton);

    //     private Supplier<Trigger> func;
    //     private Buttons(Supplier<Trigger> func) {
    //         this.func = func;
    //     }
    // }   
}