package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.FlightJoystick;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;

public class ManualDrive extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final FlightJoystick joystick;

    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    public ManualDrive(DriveTrainSubsystem driveTrain, FlightJoystick joystick) {
        this.driveTrain = driveTrain;
        this.joystick = joystick;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        // this.driveTrain.drive(this.joystick.getVerticalMovement());
        this.driveTrain.drive(this.ySpeedLimiter.calculate(this.joystick.getVerticalMovement()), this.xSpeedLimiter.calculate(this.joystick.getHorizontalMovement()), this.rotLimiter.calculate(this.joystick.getRotation()), false);
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
