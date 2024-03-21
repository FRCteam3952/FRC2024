package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.staticsubsystems.RobotGyro;

public class ClimbCommand extends Command {
    private final ClimberSubsystem climber;

    private enum ClimbStage {
        RAISING, // raising both hooks
        LOWERING_BOTH, // lower both until one makes contact
        LOWERING_ONE, // lower the one that hasn't contacted yet
        CLIMB, // climb
        WAIT
    }

    private ClimbStage stage;

    public ClimbCommand(ClimberSubsystem climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // Step 1: Raise both hooks to max
        // Step 2: Move both hooks at the same time until one has an amperage spike (chain contacted)
        // Step 3: Stop the hook that contacted, keep moving the non-contacted hook until a spike
        // Step 4: Once both hooks have contacted, raise both hooks at the same time. While raising hooks, use the gyro to correct for error
        // Step 5: Once we've reached the desired height, stop.
        
        switch(this.stage) {
            case RAISING:
                break;
            case LOWERING_BOTH:
                break;
            case LOWERING_ONE:
                break;
            case CLIMB:
                break;
            case WAIT:
                break;
            default:
                System.out.println("???? climb stage " + this.stage + " not defined");
        }
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
