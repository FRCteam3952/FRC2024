// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.Constants.PortConstants;

/**
 * Represents a swerve drive style drivetrain.
 */
public class DriveTrainSubsystem extends SubsystemBase {
    public static final double MAX_SPEED = 3.0; // 3 meters per second
    public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second


    // Location of each swerve drive, relative to motor center. +X -> moving to front of robot, +Y -> moving to left of robot. IMPORTANT.
    private final Translation2d frontLeftLocation = new Translation2d(RobotConstants.LEG_LENGTHS_M, RobotConstants.LEG_LENGTHS_M);
    private final Translation2d frontRightLocation = new Translation2d(RobotConstants.LEG_LENGTHS_M, -RobotConstants.LEG_LENGTHS_M);
    private final Translation2d backLeftLocation = new Translation2d(-RobotConstants.LEG_LENGTHS_M, RobotConstants.LEG_LENGTHS_M);
    private final Translation2d backRightLocation = new Translation2d(-RobotConstants.LEG_LENGTHS_M, -RobotConstants.LEG_LENGTHS_M);

    private final SwerveModule frontLeft = new SwerveModule(
            PortConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
            PortConstants.FRONT_LEFT_ROTATION_MOTOR_ID,
            PortConstants.FRONT_LEFT_ROTATION_CANCODER_ID,
            "fL_12"
    );
    private final SwerveModule frontRight = new SwerveModule(
            PortConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
            PortConstants.FRONT_RIGHT_ROTATION_MOTOR_ID,
            PortConstants.FRONT_RIGHT_ROTATION_CANCODER_ID,
            "fR_03"
    );
    private final SwerveModule backLeft = new SwerveModule(
            PortConstants.BACK_LEFT_DRIVE_MOTOR_ID,
            PortConstants.BACK_LEFT_ROTATION_MOTOR_ID,
            PortConstants.BACK_LEFT_ROTATION_CANCODER_ID,
            "bL_06"
    );
    private final SwerveModule backRight = new SwerveModule(
            PortConstants.BACK_RIGHT_DRIVE_MOTOR_ID,
            PortConstants.BACK_RIGHT_ROTATION_MOTOR_ID,
            PortConstants.BACK_RIGHT_ROTATION_CANCODER_ID,
            "bR_01"
    );

    private final SwerveModule[] swerveModules = {frontLeft, frontRight, backLeft, backRight};

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, RobotGyro.getRotation2d(),
            new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
    });

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DriveTrainSubsystem() {
        RobotGyro.resetGyroAngle();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // System.out.println("targets: x: " + xSpeed + " y: " + ySpeed + " rot: " + rot);
        var swerveModuleStates = kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, RobotGyro.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void drive(double speed) { // INCHES: 10 rot ~= 18.25, ~ 9 rot ~= 15.75
        frontLeft.drive(speed);
        frontRight.drive(speed);
        backLeft.drive(speed);
        backRight.drive(speed);
    }

    public void driveVoltage(double volts) {
        frontLeft.setVoltages(volts, 0);
        frontRight.setVoltages(volts, 0);
        backLeft.setVoltages(volts, 0);
        backRight.setVoltages(volts, 0);
    }

    public void consumeRawModuleStates(SwerveModuleState[] states) {
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void rotateModulesToAbsoluteZero() {
        frontLeft.rotateToAbsoluteZero();
        frontRight.rotateToAbsoluteZero();
        backLeft.rotateToAbsoluteZero();
        backRight.rotateToAbsoluteZero();
    }

    public void stop() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }
    
    @Override
    public void periodic() {
        for(SwerveModule module : swerveModules) {
            // System.out.println(module.getName() + " " + module.getDriveRotations());
            // System.out.println(module.getName() + " " + module.getPosition());
            
            // System.out.println(module.getName() + " " + TroyMathUtil.roundNearestHundredth(module.getTurningEncoderPositionConverted()));
        }
    }

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
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        odometry.update(RobotGyro.getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
    }
}
