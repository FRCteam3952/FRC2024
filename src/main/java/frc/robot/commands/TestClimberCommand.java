package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class TestClimberCommand extends Command {
    private final ClimberSubsystem climber;
    private final AbstractController joystick;

    public TestClimberCommand(ClimberSubsystem climber, AbstractController joystick) {
        this.climber = climber;
        this.joystick = joystick;

        addRequirements(climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double speed = this.joystick.getRightVerticalMovement();
        System.out.println("hoooks to: " + speed);
        // this.climber.setLeftSpeed(speed); // - is down
        // this.climber.setRightSpeed(speed); // + is down
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
