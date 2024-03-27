package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunShooterAutonCommand extends Command {
    private final ShooterSubsystem shooter;
    private final ConveyorSubsystem conveyor;
    private final double rpm, pivotAngle;
    private final Timer timer = new Timer();
    private boolean startedTimer = false;

    public RunShooterAutonCommand(ShooterSubsystem shooter, ConveyorSubsystem conveyor, double rpm, double pivotAngle) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.rpm = rpm;
        this.pivotAngle = pivotAngle;

        addRequirements(shooter, conveyor);
    }

    @Override
    public void initialize() {
        this.shooter.pivotToAngle(pivotAngle);
        this.shooter.setMotorRpm(rpm);
    }

    @Override
    public void execute() {
        if(shooter.getShooterRpm() > rpm - 75) {
            if(!startedTimer) {
                timer.restart();
                startedTimer = true;
            }
            this.conveyor.setShooterFeederMotorSpeed(1);
            this.conveyor.setConveyorMotorsSpeed(-1);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.shooter.stopShooterPID();
        this.conveyor.setShooterFeederMotorSpeed(0);
        this.conveyor.setConveyorMotorsSpeed(0);
        System.out.println("shooter done running auto");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return startedTimer && timer.get() > 1;
    }
}
