package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.controllers.AbstractController;

public class TestShooterCommand extends Command {
    private final ShooterSubsystem intake;
    private final AbstractController joystick;

    public TestShooterCommand(ShooterSubsystem intake, AbstractController joystick) {
        this.intake = intake;
        this.joystick = joystick;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setBottomMotorSpeed(-this.joystick.getRightVerticalMovement());
        System.out.println("RPM: " + intake.getShooterRpm());
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
