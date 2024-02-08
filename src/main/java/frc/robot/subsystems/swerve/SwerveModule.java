// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Flags;
import frc.robot.util.RobotMathUtil;

public class SwerveModule {
    private static final double MODULE_MAX_ANGULAR_VELOCITY = DriveTrainSubsystem.MAX_ANGULAR_SPEED * 2;
    private static final double MODULE_MAX_ANGULAR_ACCELERATION = Math.PI * 2; // radians per second squared

    private static final double MODULE_MAX_VOLTAGE_OUTPUT = 6;
    private static final double MODULE_MIN_VOLTAGE_OUTPUT = -6;

    private static final double SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG = 120;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder; // + power = CCW, - power = CW

    private final CANcoder turnAbsoluteEncoder;

    private final String name;

    // Gains are for example purposes only - must be determined for your own robot!
    private final SparkPIDController drivePIDController;
    private final SparkPIDController turnPIDController;
    // Gains are for example purposes only - must be determined for your own robot!
    
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(3, 0, 0,
            new TrapezoidProfile.Constraints(MODULE_MAX_ANGULAR_VELOCITY, MODULE_MAX_ANGULAR_ACCELERATION));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0.4, 0);

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     * @param driveMotorCANID      CAN ID for the drive motor.
     * @param turningMotorCANID    CAN ID for the turning motor.
     * @param turningEncoderCANID DIO input for the turning encoder channel A
     */
    public SwerveModule(int driveMotorCANID, int turningMotorCANID, int turningEncoderCANID, String name, boolean invertDriveMotor, boolean invertTurnMotor) {
        driveMotor = new CANSparkMax(driveMotorCANID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turningMotorCANID, MotorType.kBrushless);
        this.name = name;
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();
        turnAbsoluteEncoder = new CANcoder(turningEncoderCANID);

        driveMotor.setInverted(invertDriveMotor);
        turnMotor.setInverted(invertTurnMotor);

        this.driveMotor.setSmartCurrentLimit(20);
        this.driveMotor.setSecondaryCurrentLimit(100);

        this.turnMotor.setSmartCurrentLimit(20);
        this.turnMotor.setSecondaryCurrentLimit(100);

        // Circumference / Gear Ratio (L2 of MK4i). This evaluates to ~1.86 inches/rotation, which is close to experimental values.
        // We are therefore using the calculated value. (Thanks Ivan)
        // Since everything else is in meters, convert to meters.
        this.driveEncoder.setPositionConversionFactor(Units.inchesToMeters(4 * Math.PI / 6.75));
        this.driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(4 * Math.PI / 6.75) / 60);

        this.turnEncoder.setPositionConversionFactor(150d / 7d * Math.PI / 180 / 1.28); // ???
        this.turnEncoder.setVelocityConversionFactor(150d / 7d / 60d * Math.PI / 180 / 1.28);

        this.driveEncoder.setPosition(0);
        // this.turnEncoder.setPosition(0);
        this.turnAbsoluteEncoder.setPosition(this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());

        this.turnEncoder.setPosition(this.getTurningAbsEncoderPositionConverted());

        this.driveMotor.enableVoltageCompensation(10);
        this.drivePIDController = this.driveMotor.getPIDController();

        this.drivePIDController.setP(0.3);
        this.drivePIDController.setI(0);
        this.drivePIDController.setD(0);
        this.drivePIDController.setFF(0);
        this.drivePIDController.setOutputRange(-1, 1);

        this.turnMotor.enableVoltageCompensation(10);
        this.turnPIDController = this.turnMotor.getPIDController();

        this.turnPIDController.setPositionPIDWrappingEnabled(true);
        this.turnPIDController.setPositionPIDWrappingMinInput(-Math.PI);
        this.turnPIDController.setPositionPIDWrappingMaxInput(Math.PI);

        this.turnPIDController.setP(0.35);
        this.turnPIDController.setI(0);
        this.turnPIDController.setD(0);
        this.turnPIDController.setFF(0.02);
        this.turnPIDController.setOutputRange(-1, 1);


        //System.out.println(this.name + " inverts drive: " + this.driveMotor.getInverted() + " turn: " + this.turnMotor.getInverted());
        // System.out.println(this.name + " abs pos " + RobotMathUtil.roundNearestHundredth(this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()));
        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        // driveEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION); // EXPERIMENT LATER.

        // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        // This is the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.

        // WE NEED RADIANS I THINK
        // MAGIC NUMBER FROM DOCUMENTATION COMMENT ON METHOD COPY-PASTED. THIS SHOULD NOT BE TOUCHED UNLESS MASSIVE BREAK.
        // turningEncoder.configFeedbackCoefficient(0.087890625 * Math.PI / 180, "rad");
        // turningEncoder.setDistancePerPulse(2 * Math.PI / ENCODER_RESOLUTION); // template code, ignore.

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void resetRelativeEncodersToAbsoluteValue() {
        this.turnEncoder.setPosition(this.getTurningAbsEncoderPositionConverted());
    }

    public String getName() {
        return this.name;
    }

    public double getDriveRotations() {
        return this.driveEncoder.getPosition();
    }

    /**
     * Get the Absolute Encoder's position, converted to a usable value (in radians)
     * @return The absolute encoder's position in radians.
     */
    public double getTurningAbsEncoderPositionConverted() {
        // ORIGINAL UNITS: rotations. Converted to radians.
        return this.turnAbsoluteEncoder.getPosition().getValueAsDouble() * 360 * Math.PI / 180;
    }

    /**
     * Get the Absolute Encoder's velocity, converted to a usable value (in radians/sec)
     * @return The absolute encoder's velocity in radians per second.
     */
    public double getTurningAbsEncoderVelocityConverted() {
        // ORIGINAL UNITS: rotations per second. Converted to radians per second.
        return this.turnAbsoluteEncoder.getVelocity().getValueAsDouble() * 360 * Math.PI / 180;
    }

    public double getAbsoluteAbsolutePositionConverted() {
        // ORIGINAL UNITS: rotations. Converted to radians.
        return this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }

    /**
     * Gets the velocity of the drive encoder.
     * @return The velocity of the drive encoder, in meters/sec
     */
    public double getDriveVelocity() {
        return this.driveEncoder.getVelocity();
    }

    public double getRelativeTurnRotations() {
        return this.turnEncoder.getPosition();
    }

    public double getRelativeTurnVelocity() {
        return this.turnEncoder.getVelocity();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getAbsoluteModuleState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(this.getTurningAbsEncoderPositionConverted()));
    }

    public SwerveModuleState getAbsoluteAbsoluteModuleState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(this.getAbsoluteAbsolutePositionConverted()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getAbsoluteModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(this.getTurningAbsEncoderPositionConverted()));
    }

    public SwerveModuleState getRelativeModuleState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(this.turnEncoder.getPosition()));
    }

    public SwerveModulePosition getRelativeModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(this.turnEncoder.getPosition()));
    }

    /**
     * Directly set the voltage outputs of the motors.
     * @param driveVoltage Voltage of drive motor
     * @param turnVoltage Voltage of turn motor
     */
    public void setVoltages(double driveVoltage, double turnVoltage) {
        driveMotor.setVoltage(driveVoltage);
        turnMotor.setVoltage(turnVoltage);
    }

    /**
     * Stops the motors by sending a voltage output of 0 to both the drive and turn motors.
     */
    public void stop() {
        setVoltages(0, 0);
    }

    private double previousTurnVoltage = 0;

    public void rotateToAbsoluteZero() {
        SwerveModuleState zeroedState = new SwerveModuleState();
        this.setDesiredStateNoOptimize(zeroedState);
    }

    public void rotateToAbsoluteZero(int debugIdx) {
        SwerveModuleState zeroedState = new SwerveModuleState();
        this.setDesiredStateNoOptimize(zeroedState, debugIdx);
    }

    /**
     * ORIGINAL: {@link SwerveModuleState#optimize(SwerveModuleState, Rotation2d)}
     * <p>
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. If this is used with the PIDController class's
     * continuous input functionality, the furthest a wheel will ever rotate is {@link SwerveModule#SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG} degrees.
     *
     * <p>
     * NOTE: Team 3419 suggested this change on ChiefDelphi for mechanical reasons (sending the motors in the opposite direction at top speed is bad),
     * but we are primarily using this to resolve a separate issue where occasionally one swerve module would turn in a direction opposite of the other modules when executing a direction change of 90 degrees.
     *
     * <p>
     * ChiefDelphi comment: <a href="https://www.chiefdelphi.com/t/swerve-pid-continuous-input-and-swerve-state-optimization/416292/6">...</a>
     * <p>
     * Team 3419's code with the relevant change: <a href="https://github.com/RoHawks/UniversalSwerve/blob/2fb7c0c9c9d7d3def7ba680bcd48b4b5456f09e1/src/main/java/universalSwerve/SwerveDrive.java#L267">...</a>
     *
     * <p>
     * Change made: only optimize the rotation direction when the angle is greater than a certain amount (higher than 90deg).
     * Original value in WPILIB code: 90 degrees.
     * New value determined by {@link SwerveModule#SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG}
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     * @return Optimized swerve module state.
     */
    private static SwerveModuleState optimize(
        SwerveModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG) {
            return new SwerveModuleState(
                -desiredState.speedMetersPerSecond,
                desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = optimize(desiredState, new Rotation2d(this.getTurningAbsEncoderPositionConverted()));

        this.setDesiredStateNoOptimize(state);
    }

    public void setDesiredState(SwerveModuleState desiredState, int debugIdx) {
        SwerveModuleState state = optimize(desiredState, new Rotation2d(this.getTurningAbsEncoderPositionConverted()));

        this.setDesiredStateNoOptimize(state, debugIdx);
    }

    public void setDesiredStateNoOptimize(SwerveModuleState desiredState) {
        this.setDriveDesiredState(desiredState);
        this.setRotationDesiredState(desiredState);

        if(Math.abs(desiredState.speedMetersPerSecond) < 0.01 && Math.abs(this.getRelativeTurnVelocity()) < 0.01) {
            this.resetRelativeEncodersToAbsoluteValue();
            // System.out.println("resetting encoders");
        }
    }

    public void setDesiredStateNoOptimize(SwerveModuleState desiredState, int debugIdx) {
        this.setDriveDesiredState(desiredState);
        this.setRotationDesiredState(desiredState);

        DriveTrainSubsystem.optimizedTargetStates[debugIdx] = desiredState;

        if(Math.abs(desiredState.speedMetersPerSecond) < 0.01 && Math.abs(this.getRelativeTurnVelocity()) < 0.01) {
            this.resetRelativeEncodersToAbsoluteValue();
            // System.out.println("resetting encoders");
        }
    }

    /**
     * Sets the drive motors to follow a desired state.
     * @param optimizedDesiredState The desired module state. Should already be optimized (i.e. this method will NOT optimize them for you.)
     */
    private void setDriveDesiredState(SwerveModuleState optimizedDesiredState) {
        // Calculate the drive output from the drive PID controller.
        if(Flags.DriveTrain.ENABLED && Flags.DriveTrain.ENABLE_DRIVE_MOTORS && Flags.DriveTrain.DRIVE_PID_CONTROL) {
            drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        }
        
        // System.out.println(this.name + " velocity: " + RobotMathUtil.roundNearestHundredth(driveEncoder.getVelocity()) + " target speed: " + RobotMathUtil.roundNearestHundredth(optimizedDesiredState.speedMetersPerSecond));
        // System.out.println(this.name + ", position: " + this.driveEncoder.getPosition());
    }

    /**
     * Sets the rotation motors to follow a desired state.
     * @param optimizedDesiredState The desired module state. Should already be optimized (i.e. this method will NOT optimize them for you.)
     */
    private void setRotationDesiredState(SwerveModuleState optimizedDesiredState) {
        // System.out.println("turn encoder at: " + RobotMathUtil.roundNearestHundredth(this.turnEncoder.getPosition()) + ", abs val: " + RobotMathUtil.roundNearestHundredth(this.getTurningAbsEncoderPositionConverted()));
        if(Flags.DriveTrain.ENABLED && Flags.DriveTrain.ENABLE_TURN_MOTORS && Flags.DriveTrain.TURN_PID_CONTROL) {
            turnPIDController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);
        }

        /*
        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turningPIDController.calculate(this.getTurningAbsEncoderPositionConverted(), optimizedDesiredState.angle.getRadians());

        final double turnFeedforward = this.turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

        final double finalTurnOutput = (turnOutput + turnFeedforward);
        
        //System.out.print(this.name + " velocity: " + TroyMathUtil.roundNearestHundredth(turningEncoder.getVelocity().getValueAsDouble()) + " target speed: " + TroyMathUtil.roundNearestHundredth(state.speedMetersPerSecond));
        // System.out.print(this.name + " turning pos: " + TroyMathUtil.roundNearestHundredth(this.getTurningAbsEncoderPositionConverted() /* * 180 / Math.PI) + " target: " + TroyMathUtil.roundNearestHundredth(optimizedDesiredState.angle.getRadians()));
        // System.out.print(" rel enc: " + TroyMathUtil.roundNearestHundredth(this.turningEncoder.getPosition()));
        /*
        if(Math.abs(this.getTurningAbsEncoderPositionConverted()) > 0.02 && Math.abs(this.turningEncoder.getPosition()) > 0.02) {
            double err = TroyMathUtil.roundNearestHundredth((this.turningEncoder.getPosition() % (2 * Math.PI)) - (this.getTurningAbsEncoderPositionConverted() % (2 * Math.PI)));
            if(Math.abs(err) > 0.03) {
                System.out.print("Deviation detected between Absolute and Relative encoder values. Recalibrate?");
            }
        }
        System.out.println();*/
        /*
        System.out.println(this.name + " turn output: " + RobotMathUtil.roundNearestHundredth(finalTurnOutput));

        previousTurnVoltage = finalTurnOutput;

        if(Flags.DriveTrain.ENABLED && Flags.DriveTrain.ENABLE_TURN_MOTORS && Flags.DriveTrain.TURN_PID_CONTROL) {
            turnMotor.setVoltage(MathUtil.clamp(finalTurnOutput, MODULE_MIN_VOLTAGE_OUTPUT, MODULE_MAX_VOLTAGE_OUTPUT));
        }*/
    }

    public void directDrive(double speed) {
        if(Flags.DriveTrain.ENABLED && Flags.DriveTrain.ENABLE_DRIVE_MOTORS) {
            this.driveMotor.set(speed);
        }
    }

    public void directTurn(double speed) {
        if(Flags.DriveTrain.ENABLED && Flags.DriveTrain.ENABLE_TURN_MOTORS) {
            this.turnMotor.set(speed);
        }
    }
}
