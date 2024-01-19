package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class PS5Controller extends AbstractJoystick {
    public final CommandPS5Controller controller;

    public PS5Controller(CommandPS5Controller controller) {
        this.controller = controller;
    }
    @Override
    public double getHorizontalMovement() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getHorizontalMovement'");
    }

    @Override
    public double getVerticalMovement() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVerticalMovement'");
    }

    @Override
    public boolean getRawButtonWrapper(int button) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRawButtonWrapper'");
    }

    @Override
    public boolean getRawButtonReleasedWrapper(int button) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRawButtonReleasedWrapper'");
    }

}