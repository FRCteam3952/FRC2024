package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class CalibrateIntakeCommand extends Command {
    private final ClimberSubsystem climber;
    private final IntakeSubsystem intake;
    private final AbstractController joystick;

    public CalibrateIntakeCommand(ClimberSubsystem climber, IntakeSubsystem intake, AbstractController joystick) {
        this.climber = climber;
        this.intake = intake;

        this.joystick = joystick;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        intake.pivotToAngle(80);
        // System.out.println("Calibrating");
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
