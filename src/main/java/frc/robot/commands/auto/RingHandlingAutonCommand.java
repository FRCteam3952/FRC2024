package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.staticsubsystems.ColorSensor;

public class RingHandlingAutonCommand extends Command {
    private final ConveyorSubsystem conveyor;
    private final IntakeSubsystem intake;

    public RingHandlingAutonCommand(ConveyorSubsystem conveyor, IntakeSubsystem intake) {
        this.conveyor = conveyor;
        this.intake = intake;

        addRequirements(conveyor, intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        this.intake.setIntakeSpeed(0.75, 1);
        this.conveyor.setConveyorMotorsSpeed(-0.7);
        this.conveyor.setShooterFeederMotorSpeed(0.7);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.conveyor.setShooterFeederMotorSpeed(0);
        this.conveyor.setConveyorMotorsSpeed(0);
        this.intake.setIntakeSpeed(0);
        System.out.println("got a note auto");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return ColorSensor.isNoteColor();
    }
}
