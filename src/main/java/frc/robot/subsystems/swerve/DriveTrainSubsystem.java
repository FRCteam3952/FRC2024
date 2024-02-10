// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

/*
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
*/
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;  
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.Flags;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.Constants.PortConstants;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.Util;

/**
 * Represents a swerve drive style drivetrain.
 */
public class DriveTrainSubsystem extends SubsystemBase {
    // public static final double MAX_SPEED = 3.0; // 3 meters per second
    public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second
    private static final double SMART_OPTIMIZATION_THRESH_M_PER_SEC = 2;


    // Location of each swerve drive, relative to motor center. +X -> moving to front of robot, +Y -> moving to left of robot. IMPORTANT.
    private final Translation2d frontLeftLocation = new Translation2d(RobotConstants.LEG_LENGTHS_M, RobotConstants.LEG_LENGTHS_M);
    private final Translation2d frontRightLocation = new Translation2d(RobotConstants.LEG_LENGTHS_M, -RobotConstants.LEG_LENGTHS_M);
    private final Translation2d backLeftLocation = new Translation2d(-RobotConstants.LEG_LENGTHS_M, RobotConstants.LEG_LENGTHS_M);
    private final Translation2d backRightLocation = new Translation2d(-RobotConstants.LEG_LENGTHS_M, -RobotConstants.LEG_LENGTHS_M);

    private final SwerveModule frontLeft = new SwerveModule(
            PortConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
            PortConstants.FRONT_LEFT_ROTATION_MOTOR_ID,
            PortConstants.FRONT_LEFT_ROTATION_CANCODER_ID,
            "fL_12",
            false,
            true
    );
    private final SwerveModule frontRight = new SwerveModule(
            PortConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
            PortConstants.FRONT_RIGHT_ROTATION_MOTOR_ID,
            PortConstants.FRONT_RIGHT_ROTATION_CANCODER_ID,
            "fR_03",
            false,
            true
    );
    private final SwerveModule backLeft = new SwerveModule(
            PortConstants.BACK_LEFT_DRIVE_MOTOR_ID,
            PortConstants.BACK_LEFT_ROTATION_MOTOR_ID,
            PortConstants.BACK_LEFT_ROTATION_CANCODER_ID,
            "bL_06",
            false,
            true
    );
    private final SwerveModule backRight = new SwerveModule(
            PortConstants.BACK_RIGHT_DRIVE_MOTOR_ID,
            PortConstants.BACK_RIGHT_ROTATION_MOTOR_ID,
            PortConstants.BACK_RIGHT_ROTATION_CANCODER_ID,
            "bR_01",
            false,
            true
    );

    public final SwerveModule[] swerveModules = {frontLeft, frontRight, backLeft, backRight};

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, RobotGyro.getRotation2d(), this.getAbsoluteModulePositions(), new Pose2d());

    /**
     * Get the absolute positions of all swerve modules, using the CANCoder's *relative* mode for the module heading. Arranged as FL, FR, BL, BR
     * @return The position of all swerve modules, with module heading determined by the CANCoder's relative mode.
     */
    public SwerveModulePosition[] getAbsoluteModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getAbsoluteModulePosition(),
            frontRight.getAbsoluteModulePosition(),
            backLeft.getAbsoluteModulePosition(),
            backRight.getAbsoluteModulePosition()
        };
    }

    /**
     * The robot's current estimated pose, as estimated by the pose estimator using motor rotations and vision measurements.
     * @return The current pose of the robot.
     */
    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    /**
     * Set the robot's current pose
     * @param pose The desired current code
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(RobotGyro.getRotation2d(), this.getAbsoluteModulePositions(), pose);
    }

    /**
     * Get the current chassis speeds of the robot, relative to the robot.
     * @return The current chassis speeds of the robot, relative to the robot.
     */
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return kinematics.toChassisSpeeds(
            frontLeft.getAbsoluteModuleState(),
            frontRight.getAbsoluteModuleState(),
            backLeft.getAbsoluteModuleState(),
            backRight.getAbsoluteModuleState()
        );
    }

    public DriveTrainSubsystem() {
        RobotGyro.resetGyroAngle();

        /*
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::getRobotRelativeChassisSpeeds,
            (chassisSpeeds) -> this.consumeRawModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds)),
            new HolonomicPathFollowerConfig(3, Math.PI, new ReplanningConfig()),
            () -> !NetworkTablesUtil.getIfOnBlueTeam(),
            this
        );*/
    }

    /**
     * Get a {@link Command} to rotate all modules to absolute zero. This command will time itself out if incomplete after 1 second. Regardless of completion, all non-absolutely-absolute encoders (see {@link SwerveModule#resetEncodersToAbsolutelyAbsoluteValue()}) are set to the absolutely-absolute encoder's value.
     * @return A {@link Command} to rotate all modules to absolute zero.
     * @see {@link SwerveModule#resetEncodersToAbsolutelyAbsoluteValue()}
     */
    public Command rotateToAbsoluteZeroCommand() {
        return new RunCommand(() -> {
            frontLeft.rotateToAbsoluteZero(0);
            frontRight.rotateToAbsoluteZero(1);
            backLeft.rotateToAbsoluteZero(2);
            backRight.rotateToAbsoluteZero(3);
        }, this).until(() -> { // Until all modules have a heading of <= 0.03 rad
            for(SwerveModule module : swerveModules) {
                if(Math.abs(module.getAbsoluteModulePosition().angle.getRadians()) > 0.05) {
                    return false;
                }
            }
            return true;
        }).withTimeout(1).andThen(() -> {
            for(SwerveModule module : swerveModules) {
                module.resetEncodersToAbsolutelyAbsoluteValue();
            }
        });
    }

    //prints joystick movement 
    StructArrayPublisher<SwerveModuleState> targetSwerveStatePublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getStructArrayTopic("TargetStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> realSwerveStatePublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getStructArrayTopic("ActualStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> absoluteAbsoluteSwerveStatePublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getStructArrayTopic("AbsoluteAbsolute", SwerveModuleState.struct).publish();

    StructArrayPublisher<SwerveModuleState> relativeSwerveStatePublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getStructArrayTopic("RelativeStates", SwerveModuleState.struct).publish();
    //makes object to publish robot position relative to field
    StructPublisher<Pose2d> posePositionPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getStructTopic("estimatedOdometryPosition", Pose2d.struct).publish();
    StructPublisher<Rotation2d> robotRotationPublisher = NetworkTablesUtil.MAIN_ROBOT_TABLE.getStructTopic("rotation", Rotation2d.struct).publish();

    static SwerveModuleState[] optimizedTargetStates = new SwerveModuleState[4];

    /**
     * Method to drive the robot using joystick info.
     *
     * @param forwardSpeed  Speed of the robot in the x direction (forward).
     * @param sidewaysSpeed Speed of the robot in the y direction (sideways).
     * @param rotSpeed      Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double forwardSpeed, double sidewaysSpeed, double rotSpeed, boolean fieldRelative) {
        if(Flags.DriveTrain.ENABLED) {
            // System.out.println("targets: x: " + xSpeed + " y: " + ySpeed + " rot: " + rot);
            // System.out.println("Gyro angle: " + RobotGyro.getRotation2d().getDegrees());
            var swerveModuleStates = kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, sidewaysSpeed, rotSpeed, RobotGyro.getRotation2d()) : new ChassisSpeeds(forwardSpeed, sidewaysSpeed, rotSpeed));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ManualDriveCommand.MAX_SPEED_METERS_PER_SEC);

            boolean shouldOptimize = true;
            for (SwerveModule swerveModule : swerveModules) {
                if (Math.abs(swerveModule.getDriveVelocity()) > SMART_OPTIMIZATION_THRESH_M_PER_SEC) {
                    shouldOptimize = false;
                    break;
                }
            }
            if(shouldOptimize || !Flags.DriveTrain.SPEED_BASED_SWERVE_MODULE_OPTIMIZATION) {
                // System.out.println("Optimizing");
                frontLeft.setDesiredState(swerveModuleStates[0], 0);
                frontRight.setDesiredState(swerveModuleStates[1], 1);
                backLeft.setDesiredState(swerveModuleStates[2], 2);
                backRight.setDesiredState(swerveModuleStates[3], 3);
            } else {
                System.out.println("Not Optimizing");
                frontLeft.setDesiredStateNoOptimize(swerveModuleStates[0], 0);
                frontRight.setDesiredStateNoOptimize(swerveModuleStates[1], 1);
                backLeft.setDesiredStateNoOptimize(swerveModuleStates[2], 2);
                backRight.setDesiredStateNoOptimize(swerveModuleStates[3], 3);
            }

            targetSwerveStatePublisher.set(optimizedTargetStates);
        }
    }

    /**
     * Set all drive motor speeds to a specified value without any PID control.
     * @param speed The desired speed, [-1, 1]
     */
    public void directDriveSpeed(double speed) { // INCHES: 10 rot ~= 18.25, ~ 9 rot ~= 15.75
        frontLeft.directDrive(speed);
        frontRight.directDrive(speed);
        backLeft.directDrive(speed);
        backRight.directDrive(speed);
    }

    /**
     * Set all turn motor speeds to a specified value without any PID control.
     * @param speed The desired speed, [-1, 1]
     */
    public void directTurnSpeed(double speed) {
        frontLeft.directTurn(speed);
        frontRight.directTurn(speed);
        backLeft.directTurn(speed);
        backRight.directTurn(speed);
    }

    /**
     * Set all drive motor voltages to a specified value without any PID control.
     * @param speed The desired voltage output
     */
    public void directDriveVoltage(double volts) {
        frontLeft.setVoltages(volts, 0);
        frontRight.setVoltages(volts, 0);
        backLeft.setVoltages(volts, 0);
        backRight.setVoltages(volts, 0);
    }

    /**
     * Consumes a set of raw {@link SwerveModuleState}s, setting the modules to target those states.
     * @param states An array of 4 desired states, in the order FL, FR, BL, BR.
     */
    public void consumeRawModuleStates(SwerveModuleState[] states) {
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    /**
     * Stops all motors, driving or turning, by sending a target voltage of 0.
     */
    public void stop() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }
    
    @Override
    public void periodic() {
        //publishes each wheel information to network table for debugging
        realSwerveStatePublisher.set(new SwerveModuleState[]{frontLeft.getAbsoluteModuleState(), frontRight.getAbsoluteModuleState(), backLeft.getAbsoluteModuleState(), backRight.getAbsoluteModuleState()});
        absoluteAbsoluteSwerveStatePublisher.set(new SwerveModuleState[]{frontLeft.getAbsoluteAbsoluteModuleState(), frontRight.getAbsoluteAbsoluteModuleState(), backLeft.getAbsoluteAbsoluteModuleState(), backRight.getAbsoluteAbsoluteModuleState()});
        relativeSwerveStatePublisher.set(new SwerveModuleState[]{frontLeft.getRelativeModuleState(), frontRight.getRelativeModuleState(), backLeft.getRelativeModuleState(), backRight.getRelativeModuleState()});
        //posts robot position to network table
        posePositionPublisher.set(this.getPose());
        robotRotationPublisher.set(RobotGyro.getRotation2d());

        //noinspection StatementWithEmptyBody
        for (SwerveModule module : swerveModules) {
            // System.out.println(module.getName() + " rel vel: " + RobotMathUtil.roundNearestHundredth(module.getRelativeTurnVelocity()) + ", abs vel: " + RobotMathUtil.roundNearestHundredth(module.getTurningAbsEncoderVelocityConverted()));
            // System.out.println(module.getName() + " " + module.getDriveRotations());
            // System.out.println(module.getName() + " " + module.getPosition());

            // System.out.println(module.getName() + " " + TroyMathUtil.roundNearestHundredth(module.getTurningEncoderPositionConverted()));
        }

        this.updateOdometry();
        // System.out.println(this.getPose());
    }

    /**
     * Generates a command to follow a given trajectory, then stops at the end if desired.
     * @param trajectory The desired trajectory to follow.
     * @param stopOnEnd Whether to set speeds to 0 at the end.
     * @return
     */
    public Command generateTrajectoryFollowerCommand(Trajectory trajectory, boolean stopOnEnd) {
        return new SwerveControllerCommand(
            trajectory,
            this::getPose,
            this.kinematics,
            new PIDController(DriveConstants.TRAJ_X_CONTROLLER_KP, 0, 0),
            new PIDController(DriveConstants.TRAJ_Y_CONTROLLER_KP, 0, 0),
            new ProfiledPIDController(DriveConstants.TRAJ_THETA_CONTROLLER_KP, 0, 0, new TrapezoidProfile.Constraints(DriveConstants.TRAJ_MAX_ANG_VELO, DriveConstants.TRAJ_MAX_ANG_ACCEL)),
            this::consumeRawModuleStates,
            this
        ).andThen(() -> {
            if (stopOnEnd) {
                drive(0, 0, 0, false);
            }
        });
    }

    /**
     * Updates the field relative position of the robot using module state readouts.
     */
    public void updateOdometry() {
        this.poseEstimator.update(RobotGyro.getRotation2d(), new SwerveModulePosition[]{frontLeft.getAbsoluteModulePosition(), frontRight.getAbsoluteModulePosition(), backLeft.getAbsoluteModulePosition(), backRight.getAbsoluteModulePosition()});
    }

    /**
     * Updates the field relative position of the robot using vision measurements.
     */
    public void updateOdometryWithVision() {
        // this.poseEstimator.addVisionMeasurement();
    }
}
