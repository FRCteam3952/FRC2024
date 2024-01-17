package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.NintendoProController;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;

public class ManualDrive extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final NintendoProController joystick;

    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    private static final double MAX_SPEED_METERS_PER_SEC = 3.5;

    public ManualDrive(DriveTrainSubsystem driveTrain, NintendoProController joystick) {
        this.driveTrain = driveTrain;
        this.joystick = joystick;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // System.out.println("vert: " + this.joystick.getRightVerticalMovement() + ", hor: " + this.joystick.getRightHorizontalMovement());
        // this.driveTrain.drive(this.joystick.getVerticalMovement());
        this.driveTrain.drive(this.ySpeedLimiter.calculate(this.joystick.getRightVerticalMovement()) * MAX_SPEED_METERS_PER_SEC, this.xSpeedLimiter.calculate(this.joystick.getRightHorizontalMovement()) * MAX_SPEED_METERS_PER_SEC, this.rotLimiter.calculate(this.joystick.getLeftHorizontalMovement()), false);
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
