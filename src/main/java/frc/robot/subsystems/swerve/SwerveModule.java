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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TroyMathUtil;

public class SwerveModule {
    private static final double MODULE_MAX_ANGULAR_VELOCITY = DriveTrainSubsystem.MAX_ANGULAR_SPEED * 2;
    private static final double MODULE_MAX_ANGULAR_ACCELERATION = Math.PI * 2; // radians per second squared

    private static final double SWERVE_MODULE_PLS_NO_EXPLODE_MAX_SPEED = 6;
    private static final double SWERVE_MODULE_PLS_NO_EXPLODE_MIN_SPEED = -6;

    private static final double SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG = 120;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final CANcoder turningAbsEncoder;

    private final String name;

    // Gains are for example purposes only - must be determined for your own robot!
    private final SparkPIDController drivePIDController; //  = new PIDController(5.7, 0, 0);
    // private final SparkPIDController turningPIDController;
    // Gains are for example purposes only - must be determined for your own robot!
    
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(17.5, 0, 0,
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
    public SwerveModule(int driveMotorCANID, int turningMotorCANID, int turningEncoderCANID, String name) {
        driveMotor = new CANSparkMax(driveMotorCANID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorCANID, MotorType.kBrushless);
        this.name = name;
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        turningAbsEncoder = new CANcoder(turningEncoderCANID);

        // Circumference / Gear Ratio (L2 of MK4i). This evaluates to ~1.86 inches/rotation, which is close to experimental values.
        // We are therefore using the calculated value. (Thanks Ivan)
        // Since everything else is in meters, convert to meters.
        this.driveEncoder.setPositionConversionFactor(Units.inchesToMeters(4 * Math.PI / 6.75));
        this.driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(4 * Math.PI / 6.75) / 60);

        this.turningEncoder.setPositionConversionFactor(150d / 7 * Math.PI / 180);
        this.turningEncoder.setVelocityConversionFactor(150d / 7d / 60d * Math.PI / 180);

        // this.turningMotor.setInverted(true);

        this.driveEncoder.setPosition(0);
        this.turningEncoder.setPosition(0);
        this.turningAbsEncoder.setPosition(this.turningAbsEncoder.getAbsolutePosition().getValueAsDouble());

        this.driveMotor.enableVoltageCompensation(10);
        this.drivePIDController = this.driveMotor.getPIDController();

        this.drivePIDController.setP(0.5);
        this.drivePIDController.setI(0);
        this.drivePIDController.setD(0);
        this.drivePIDController.setFF(0);
        this.drivePIDController.setOutputRange(-1, 1);

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

    public String getName() {
        return this.name;
    }

    public double getDriveRotations() {
        return this.driveEncoder.getPosition();
    }

    public double getTurningAbsEncoderPositionConverted() {
        // WE NEED RADIANS I THINK
        // MAGIC NUMBER FROM DOCUMENTATION COMMENT ON METHOD COPY-PASTED. THIS SHOULD NOT BE TOUCHED UNLESS MASSIVE BREAK.
        // turningEncoder.configFeedbackCoefficient(0.087890625 * Math.PI / 180, "rad");

        // They changed the API on me so we have to do this now.
        // System.out.println("UNITS: " + this.turningEncoder.getAbsolutePosition().getUnits());

        // UNITS is a rotation.
        double encoderValue = this.turningAbsEncoder.getPosition().getValueAsDouble();
        return encoderValue * 360 * Math.PI / 180;
    }

    public double getTurningAbsEncoderVelocityConverted() {
        // WE NEED RADIANS I THINK
        // MAGIC NUMBER FROM DOCUMENTATION COMMENT ON METHOD COPY-PASTED. THIS SHOULD NOT BE TOUCHED UNLESS MASSIVE BREAK.
        // turningEncoder.configFeedbackCoefficient(0.087890625 * Math.PI / 180, "rad");

        // They changed the API on me so we have to do this now.
        // System.out.println("UNITS: " + this.turningEncoder.getAbsolutePosition().getUnits());

        // UNITS is a rotation per second.
        return this.turningAbsEncoder.getVelocity().getValueAsDouble() * 360 * Math.PI / 180;
    }

    public double getDriveVelocity() {
        return this.driveEncoder.getVelocity();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(this.getTurningAbsEncoderPositionConverted()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(this.getTurningAbsEncoderPositionConverted()));
    }

    public void setVoltages(double driveVoltage, double turnVoltage) {
        driveMotor.setVoltage(driveVoltage);
        turningMotor.setVoltage(turnVoltage);
    }

    public void stop() {
        setVoltages(0, 0);
    }

    private double previousTurnVoltage = 0;

    public void rotateToAbsoluteZero() {
        SwerveModuleState zeroedState = new SwerveModuleState();
        this.setDesiredStateNoOptimize(zeroedState);
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. If this is used with the PIDController class's
     * continuous input functionality, the furthest a wheel will ever rotate is {@link SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG} degrees.
     *
     * <p>
     * NOTE: Team 3419 suggested this change on ChiefDelphi for mechanical reasons (sending the motors in the opposite direction at top speed is bad),
     * but we are primarily using this to resolve a separate issue where occasionally one swerve module would turn in a direction opposite of the other modules when executing a direction change of 90 degrees.
     * 
     * <p>
     * Change made: only optimize the rotation direction when the angle is greater than a certain amount (higher than 90deg).
     * Original value in WPILIB code: 90 degrees.
     * New value determined by {@link SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG}
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

        this.setDesiredStateNoOptimize(state);
        DriveTrainSubsystem.optimizedTargetStates[debugIdx] = state;
    }

    private void setDesiredStateNoOptimize(SwerveModuleState desiredState) {
        this.setDriveDesiredState(desiredState);
        this.setRotationDesiredState(desiredState);
    }

    /**
     * Sets the drive motors to follow a desired state.
     * @param optimizedDesiredState The desired module state. Should already be optimized (i.e. this method will NOT optimize them for you.)
     */
    private void setDriveDesiredState(SwerveModuleState optimizedDesiredState) {
        // Calculate the drive output from the drive PID controller.

        drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        
        // System.out.println(this.name + " velocity: " + TroyMathUtil.roundNearestHundredth(driveEncoder.getVelocity()) + " target speed: " + TroyMathUtil.roundNearestHundredth(optimizedDesiredState.speedMetersPerSecond));
        // System.out.println(this.name + ", position: " + this.driveEncoder.getPosition());
    }

    /**
     * Sets the rotation motors to follow a desired state.
     * @param optimizedDesiredState The desired module state. Should already be optimized (i.e. this method will NOT optimize them for you.)
     */
    private void setRotationDesiredState(SwerveModuleState optimizedDesiredState) {
        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turningPIDController.calculate(this.getTurningAbsEncoderPositionConverted(), optimizedDesiredState.angle.getRadians());

        final double turnFeedforward = this.turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

        final double finalTurnOutput = -(turnOutput + turnFeedforward);
        
        //System.out.print(this.name + " velocity: " + TroyMathUtil.roundNearestHundredth(turningEncoder.getVelocity().getValueAsDouble()) + " target speed: " + TroyMathUtil.roundNearestHundredth(state.speedMetersPerSecond));
        System.out.print(this.name + " turning pos: " + TroyMathUtil.roundNearestHundredth(this.getTurningAbsEncoderPositionConverted() /** 180 / Math.PI*/) + " target: " + TroyMathUtil.roundNearestHundredth(optimizedDesiredState.angle.getRadians()));
        System.out.print(" rel enc: " + this.turningEncoder.getPosition());
        System.out.println(/*this.name + */" turn output: " + TroyMathUtil.roundNearestHundredth(previousTurnVoltage));

        previousTurnVoltage = finalTurnOutput;
        turningMotor.setVoltage(MathUtil.clamp(finalTurnOutput, SWERVE_MODULE_PLS_NO_EXPLODE_MIN_SPEED, SWERVE_MODULE_PLS_NO_EXPLODE_MAX_SPEED));
    }

    public void drive(double speed) {
        this.driveMotor.set(speed);
    }
}
