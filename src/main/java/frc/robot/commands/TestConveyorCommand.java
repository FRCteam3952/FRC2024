package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.conveyor.ConveyorSubsystem;
import frc.robot.controllers.AbstractController;

public class TestConveyorCommand extends Command {
    private final ConveyorSubsystem conveyor;
    private final AbstractController joystick;

    public TestConveyorCommand(ConveyorSubsystem conveyor, AbstractController joystick) {
        this.conveyor = conveyor;
        this.joystick = joystick;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        conveyor.setShooterFeederMotorSpeed(-this.joystick.getRightVerticalMovement());

        // this.intake.setPivotSpeed(-joystick.getRightVerticalMovement());

        // System.out.println("intake position: " + Util.nearestHundredth(intake.getPivotPosition()));

        // System.out.println("lower intake current: " + this.intake.getFollowerMotorCurrent() + ", top current: " + this.intake.getLeaderMotorCurrent());
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
