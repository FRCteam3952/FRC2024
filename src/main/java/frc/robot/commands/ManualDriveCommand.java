package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Flags;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.ControlHandler;
import frc.robot.util.Util;

public class ManualDriveCommand extends Command {
    public static final double MAX_SPEED_METERS_PER_SEC = Flags.DriveTrain.LOWER_MAX_SPEED ? 1.5 : 3;
    private final DriveTrainSubsystem driveTrain;
    private final AbstractController joystick;
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.5);
    private final Trigger autoAimSubwoofer;

    public ManualDriveCommand(DriveTrainSubsystem driveTrain, AbstractController joystick) {
        this.driveTrain = driveTrain;
        this.joystick = joystick;
        this.autoAimSubwoofer = ControlHandler.get(joystick, ControllerConstants.AUTO_AIM_FOR_SHOOT);

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        // this.driveTrain.setPose(new Pose2d(2, 7, RobotGyro.getRotation2d()));
    }

    private double flipFactor() {
        if(Util.onBlueTeam()) {
            return 1;
        }
        return -1;
    }

    @Override
    public void execute() {
        // System.out.println("vert: " + this.joystick.getRightVerticalMovement() + ", hor: " + this.joystick.getRightHorizontalMovement());
        // this.driveTrain.drive(this.joystick.getVerticalMovement());
        double flip = flipFactor();
        double ySpeed = Util.squareKeepSign(this.ySpeedLimiter.calculate(-this.joystick.getLeftVerticalMovement() * flip)) * MAX_SPEED_METERS_PER_SEC;
        double xSpeed = Util.squareKeepSign(this.xSpeedLimiter.calculate(-this.joystick.getLeftHorizontalMovement() * flip)) * MAX_SPEED_METERS_PER_SEC;
        double rotSpeed = -this.joystick.getRightHorizontalMovement() * 1.2;

        if(autoAimSubwoofer.getAsBoolean()) {
            Rotation2d angleToSubwooferTarget = directionToSubwooferTarget();
            Rotation2d robotHeading = RobotGyro.getRotation2d();
            double headingDeg = 180 + Util.bringAngleWithinUnitCircle(robotHeading.getDegrees());
            double rotateByAmount = headingDeg - angleToSubwooferTarget.getDegrees();
            if(rotateByAmount > 180) {
                rotateByAmount -= 360;
            }
            rotateByAmount = -rotateByAmount;
            if(rotateByAmount < -180) {
                rotateByAmount += 360;
            }
            double rotSpeed2 = MathUtil.clamp(3 * (Math.toRadians(rotateByAmount)), -1.7, 1.7);
            System.out.println("angle to subwoofer target: " + directionToSubwooferTarget() + ", rotating " + rotateByAmount + " at a speed of " + rotSpeed2 + " to get there");
            //System.out.println("current rot: " + RobotGyro.getRotation2d());
            rotSpeed = rotSpeed2;
        }

        // System.out.println("forward speed: " + ySpeed + ", x speed: " + xSpeed);
        // System.out.println("y: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftVerticalMovement()) + ", x: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftHorizontalMovement()));
        this.driveTrain.drive(ySpeed, xSpeed, rotSpeed, true);
    }

    /**
     * Calculate the angle the gyroscope should be at in order to look at the speaker
     * 
     * @return A Rotation2d representing the angle to the speaker. The gyroscope value should equal this value when the robot is facing the speaker.
     */
    private Rotation2d directionToSubwooferTarget() {
        // get the target for our alliance color
        Pose3d targetPose;
        if(Util.onBlueTeam()) {
            // our target is tag 7
            targetPose = Util.getTagPose(7);
        } else {
            // our target is tag 4
            targetPose = Util.getTagPose(4);
        }

        // now we know where to aim, compare our current location with our target
        Pose2d targetPose2d = targetPose.toPose2d();
        Pose2d robotPos = this.driveTrain.getPose();
        // tan(theta) = opp/adj
        // theta = atan(opp/adj)

        double theta = Math.atan2(targetPose2d.getY() - robotPos.getY(), targetPose2d.getX() - robotPos.getX()); // trust me bro
        return new Rotation2d(theta);
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
