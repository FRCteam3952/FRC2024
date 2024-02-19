package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.staticsubsystems.RobotGyro;

/**
 * A pose estimator based off of gyroscope accelerometer readings.
 */
public class GyroPoseEstimator {
    private static final double DELTA_TIME_S = 0.02; // 20ms

    private Translation3d position;

    private Translation3d velocity;

    public GyroPoseEstimator() {
        this(new Translation3d());
    }

    public GyroPoseEstimator(Translation3d initialPosition) {
        this.position = initialPosition;

        this.velocity = new Translation3d();
    }

    /**
     * Updates the pose. This method should be called on every robot tick.
     */
    public void update() {
        Translation3d accels = new Translation3d(RobotGyro.getAccelX(), RobotGyro.getAccelY(), RobotGyro.getAccelZ()).rotateBy(new Rotation3d(0, 0, RobotGyro.getRotation2d().getRadians()));
        this.velocity = this.velocity.plus(accels.times(DELTA_TIME_S));
        this.position = this.velocity.times(DELTA_TIME_S);
    }

    public Translation3d getPosition() {
        return this.position;
    }

    public void resetPosition() {
        this.setPosition(new Translation3d());
    }

    public void setPosition(Translation3d newPos) {
        this.position = newPos;
    }

    public Translation3d getVelocity() {
        return this.velocity;
    }

    public void resetVelocity() {
        this.velocity = new Translation3d();
    }
}
