package frc.robot.subsystems.staticsubsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

import static edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

/**
 * Wrapper around gyro
 */

public final class RobotGyro {
    private static final ADIS16470_IMU gyro = new ADIS16470_IMU();
    private static double angleAdjust = 0;

    static {
        // gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kZ);
        gyro.calibrate();
        gyro.reset();
    }

    private RobotGyro() {
    }

    /**
     * make sure this class is instantiated properly by poking it
     */
    public static void poke() {
        System.out.println("RobotGyro init");
    }

    /**
     * Get the robot's current rotation as a {@link Rotation2d}
     *
     * @return The robot's current rotation as a {@link Rotation2d}
     */
    public static Rotation2d getRotation2d() {
        return new Rotation2d(-Math.toRadians(gyro.getAngle(IMUAxis.kZ) + angleAdjust));
    }

    /**
     * Get the robot's current yaw value.
     *
     * @return The robot's current yaw value.
     */
    public static double getGyroAngleDegreesYaw() {
        return gyro.getAngle(IMUAxis.kZ) + angleAdjust;
    }

    /**
     * Get the robot's current roll value.
     *
     * @return The robot's current roll value.
     */
    public static double getGyroAngleDegreesRoll() {
        return gyro.getXComplementaryAngle() + angleAdjust;
    }

    /**
     * Get the robot's current pitch value.
     *
     * @return The robot's current pitch value.
     */
    public static double getGyroAngleDegreesPitch() {
        return gyro.getYComplementaryAngle() + angleAdjust;
    }

    public static double getAccelX() {
        return gyro.getAccelX();
    }

    public static double getAccelY() {
        return gyro.getAccelY();
    }

    public static double getAccelZ() {
        return gyro.getAccelZ();
    }

    /**
     * Reset the gyro such that the current heading is equal to 0.
     */
    public static void resetGyroAngle() {
        gyro.reset();
        angleAdjust = 0;
    }

    /**
     * Set the gyro's current heading to a specific value.
     *
     * @param deg The desired current heading, in degrees.
     */
    public static void setGyroAngle(double deg) {
        resetGyroAngle();
        angleAdjust = -deg;
    }

    /**
     * Rerun gyro calibration. The robot should not be moving, or need to be moving soon, when this occurs.
     */
    public static void robotCalibrate() {
        gyro.calibrate();
    }
}
