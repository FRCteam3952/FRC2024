package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Flags;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.RobotMathUtil;

public class ManualDriveCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final AbstractController joystick;

    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2);

    public static final double MAX_SPEED_METERS_PER_SEC = Flags.DriveTrain.LOWER_MAX_SPEED ? 3 : 5; 

    public ManualDriveCommand(DriveTrainSubsystem driveTrain, AbstractController joystick) {
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
        double ySpeed = RobotMathUtil.squareKeepSign(this.ySpeedLimiter.calculate(-this.joystick.getLeftVerticalMovement())) * MAX_SPEED_METERS_PER_SEC;
        double xSpeed = RobotMathUtil.squareKeepSign(this.xSpeedLimiter.calculate(-this.joystick.getLeftHorizontalMovement())) * MAX_SPEED_METERS_PER_SEC;
        double rotSpeed = this.rotLimiter.calculate(-this.joystick.getRightHorizontalMovement());
        // System.out.println("forward speed: " + ySpeed + ", x speed: " + xSpeed);
        // System.out.println("y: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftVerticalMovement()) + ", x: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftHorizontalMovement()));
        this.driveTrain.drive(ySpeed, xSpeed, rotSpeed, true);
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
