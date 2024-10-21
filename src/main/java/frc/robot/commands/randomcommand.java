package frc.robot.commands;

// Importing in any needed libraries
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// Start of command
public class randomcommand extends Command {
    public ShooterSubsystem shooter;
    public Timer timer;

    public randomcommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.timer = new Timer(); 
        addRequirements(shooter);
    }
    // Resetting the timer
    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        this.shooter.pivotToAngle(70);
    }

    @Override // Called when command ends
    public void end(boolean interrupted) {

    }

    @Override // Displays the time that the command was enabled for
    public boolean isFinished() {
        return timer.hasElapsed(1.5);
    } 
}