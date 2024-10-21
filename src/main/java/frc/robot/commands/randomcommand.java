package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class randomcommand extends Command {
    public ShooterSubsystem shooter;
    public Timer timer;

    public randomcommand(ShooterSubsystem system) {
        this.shooter = shooter;
        this.timer = new Timer();
        addRequirements(shooter);
    }
    // Resetting the timer
    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        this.shooter.pivotToAngle(45);
    }

    @Override // Called when command ends
    public void end(boolean interrupted) {

    }

    @Override // Displays the time that the command was enabled for
    public boolean isFinished() {
        return timer.hasElapsed(1.5);
    } 
}