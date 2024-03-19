package frc.robot.commands;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.NetworkTablesUtil;

public class TestShooterCommand extends Command {
    private static final DoublePublisher rpmPub = NetworkTablesUtil.MAIN_ROBOT_TABLE.getDoubleTopic("shooter_rpm").publish();
    private final ShooterSubsystem shooter;
    private final AbstractController joystick;

    public TestShooterCommand(ShooterSubsystem shooter, AbstractController joystick) {
        this.shooter = shooter;
        this.joystick = joystick;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // shooter.setBottomMotorSpeed(-this.joystick.getRightVerticalMovement());
        if (joystick.rightShoulderTrigger().getAsBoolean()) {
            shooter.setMotorRpm(700);
        } else {
            shooter.stopShooterPID();
        }
        // System.out.println("RPM: " + shooter.getShooterRpm());
        rpmPub.set(shooter.getShooterRpm());
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
