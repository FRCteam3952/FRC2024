package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ExampleCommand extends Command {
    private final ShooterSubsystem shooter;
    private final Timer timer = new Timer();
    private boolean startedTimer = false;

    public ExampleCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        startedTimer = false;
        this.shooter.pivotToAngle(0);
        System.out.println("ExampleCommand start");
    }

    @Override
    public void execute() {
        if (this.timer.hasElapsed(3.0)) {
            this.shooter.pivotToAngle(70);
        } else {
            this.shooter.pivotToAngle(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(6.0);
    }
}
