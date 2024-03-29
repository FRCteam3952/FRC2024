package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Flags;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.AprilTagHandler;
import frc.robot.util.ControlHandler;
import frc.robot.util.Util;

import java.util.Optional;

public class ManualDriveCommand extends Command {
    public static final double MAX_SPEED_METERS_PER_SEC = Flags.DriveTrain.LOWER_MAX_SPEED ? 1.5 : 3;
    private final DriveTrainSubsystem driveTrain;
    private final AbstractController joystick;
    private final AprilTagHandler aprilTagHandler;
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.5);
    private final Trigger autoAimSubwoofer;
    private final LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public ManualDriveCommand(DriveTrainSubsystem driveTrain, AbstractController joystick, AprilTagHandler aprilTagHandler) {
        this.driveTrain = driveTrain;
        this.joystick = joystick;
        this.aprilTagHandler = aprilTagHandler;
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
//        double rotSpeed = -this.joystick.getRightHorizontalMovement() * 3.5;

        // this Should use option filtering, in the future...
//        Optional<Rotation2d> angleToSubwooferTargetOption = directionToSubwooferTarget();

        double rotSpeed = directionToSubwooferTarget()
                .flatMap(angleToSubwooferTarget -> {
                    if(false && autoAimSubwoofer.getAsBoolean()) {
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
                        return Optional.of(rotSpeed2);
                    } else {
                        // reset the filter as soon as manual control is re-enabled
                        filter.reset();
                        return Optional.empty();
                    }
                })
                .orElse(-this.joystick.getRightHorizontalMovement() * 3.5);

        // System.out.println("forward speed: " + ySpeed + ", x speed: " + xSpeed);
        // System.out.println("y: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftVerticalMovement()) + ", x: " + RobotMathUtil.roundNearestHundredth(this.joystick.getLeftHorizontalMovement()));
        this.driveTrain.drive(ySpeed, xSpeed, rotSpeed, true);
    }

    /**
     * Calculate the angle the gyroscope should be at in order to look at the speaker
     * 
     * @return A Rotation2d representing the angle to the speaker. The gyroscope value should equal this value when the robot is facing the speaker.
     */
    private Optional<Rotation2d> directionToSubwooferTarget() {
        int tagId;
        if (Util.onBlueTeam()) {
            tagId = 7;
        } else {
            tagId = 4;
        }

        // i love Optional<T> :3
        Optional<Rotation2d> directionToSubwooferTarget = aprilTagHandler
                .getJetsonAprilTagPoses()
                .stream()
                .filter((tag) -> tag.tagId() == tagId)
                .findFirst()
                .map(AprilTagHandler.RobotPoseAndTagDistance::fieldRelativePose)
                .map((robotPose) -> {
                    Pose2d targetPose2d = Util.getTagPose(tagId).toPose2d();

                    // now we know where to aim, compare our current location with our target
                    return Math.atan2(
                            targetPose2d.getY() - robotPose.getY(),
                            targetPose2d.getX() - robotPose.getX()
                    ); // trust me bro
                })
                .map(filter::calculate)
                .map(Rotation2d::new);

        return directionToSubwooferTarget;
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
