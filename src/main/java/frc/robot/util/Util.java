package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.CoordinateAxis;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.function.Supplier;

/**
 * Does stuff for us
 */
public final class Util {
    public static final CoordinateSystem JETSON_APRILTAGS_COORD_SYSTEM = new CoordinateSystem(CoordinateAxis.E(), CoordinateAxis.U(), CoordinateAxis.N());
    public static final AprilTagFieldLayout TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private Util() {
        throw new UnsupportedOperationException("Util is a utility class and should not be instantiated!");
    }

    /**
     * Bring a degree angle to a value between [0, 360)
     *
     * @param angleDeg an angle in degrees
     * @return an equivalent angle, between [0, 360)
     */
    public static double bringAngleWithinUnitCircle(double angleDeg) {
        while (angleDeg < 0) {
            angleDeg += 360;
        }
        while (angleDeg >= 360) {
            angleDeg -= 360;
        }
        return angleDeg;
    }

    /**
     * Calculates the distance between two points on a 2D plane.
     *
     * @param x1 First point's x coordinate
     * @param x2 Second point's x coordinate
     * @param y1 First point's y coordinate
     * @param y2 Second point's y coordinate
     * @return The distance
     */
    public static double distance(double x1, double x2, double y1, double y2) { //2d distance calc
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
    }

    /**
     * Returns the distance between two points on a 3D plane
     *
     * @param x1 First point's x coordinate
     * @param x2 Second point's x coordinate
     * @param y1 First point's y coordinate
     * @param y2 Second point's y coordinate
     * @param z1 First point's z coordinate
     * @param z2 Second point's z coordinate
     * @return The distance
     */
    public static double distance(double x1, double x2, double y1, double y2, double z1, double z2) { //3d distance calc
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2) + Math.pow(z1 - z2, 2));
    }

    /**
     * Solves Law of Cosines for the angle
     *
     * @param a Side a
     * @param b Side b
     * @param c Side c
     * @return Angle opposite c
     */
    public static double lawOfCosinesForAngle(double a, double b, double c) { //law of cosines calc
        return Math.toDegrees(Math.acos((Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2)) / (2 * (a * b))));
    }

    /**
     * Solves Law of Cosines for a side
     *
     * @param a                Side a
     * @param b                Side b
     * @param includedAngleDeg Angle opposite the unknown side
     * @return The side you're looking for
     */
    public static double lawOfCosinesForSide(double a, double b, double includedAngleDeg) {
        return Math.sqrt(a * a + b * b - 2 * a * b * Math.cos(Math.toRadians(includedAngleDeg)));
    }

    /**
     * Solves Law of Sines for a side
     *
     * @param angle The angle opposite side A
     * @param a     Side opposite angle
     * @param b     Side opposite unknown angle
     * @return Unknown angle opposite b
     */
    public static double lawOfSinesForAngle(double angle, double a, double b) { //law of sines calc
        return Math.toDegrees(Math.asin((b * Math.sin(Math.toRadians(angle))) / a));
    }

    /**
     * Returns the angle between two points on a 3D plane
     *
     * @param x1 First point's x coordinate
     * @param x2 Second point's x coordinate
     * @param y1 First point's y coordinate
     * @param y2 Second point's y coordinate
     * @param z1 First point's z coordinate
     * @param z2 Second point's z coordinate
     * @return The angle between the points
     */
    public static double angleBetweenLines(double x1, double y1, double z1, double x2, double y2, double z2) { //angle between lines
        double dotProduct = x1 * x2 + y1 * y2 + z1 * z2;
        return Math.toDegrees(Math.acos(dotProduct / (Math.abs(distance(0, x1, 0, y1, 0, z1) * distance(0, x2, 0, y2, 0, z2)))));
    }

    /**
     * Rotates a point around the origin
     *
     * @param x     X coordinate
     * @param y     Y coordinate
     * @param angle Angle to rotate by, in deg
     */
    public static double[] rotatePoint(double x, double y, double angle) {
        double[] rotatedPoint = new double[2];
        rotatedPoint[0] = x * Math.cos(Math.toRadians(angle)) - y * Math.sin(Math.toRadians(angle));
        rotatedPoint[1] = x * Math.sin(Math.toRadians(angle)) + y * Math.cos(Math.toRadians(angle));
        return rotatedPoint;
    }

    /**
     * Rounds a double to the nearest hundredth.
     *
     * @param d The value to round.
     * @return The parameter, rounded to the nearest hundredth place.
     */
    public static double nearestHundredth(double d) {
        return Math.floor(d * 100) / 100d;
    }

    /**
     * Square a value but keep the sign of the value. Ex: squareKeepSign(-1) = -1
     *
     * @param d The input value
     * @return The input value squared, but keeping the same sign.
     */
    public static double squareKeepSign(double d) {
        return d * d * Math.signum(d);
    }

    /**
     * If a given flag is true, runs the supplier. Otherwise returns null.
     *
     * @param objSupplier A supplier (should be a constructor call) for a class.
     * @param flag        If true, runs the supplier. Otherwise the supplier is not run and null is returned.
     * @return The return value of the supplier if flag is true, else null.
     */
    public static <T> T createIfFlagElseNull(Supplier<T> objSupplier, boolean flag) {
        if (flag) {
            return objSupplier.get();
        }
        return null;
    }

    /**
     * Gets whether the robot is on the blue alliance, according to the Driver Station.
     *
     * @return True if on blue, false if on red. If alliance is not present, will default to true.
     */
    public static boolean onBlueTeam() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get().equals(Alliance.Blue);
        }
        System.out.println("Alliance not present!");
        DriverStation.reportError("Alliance not present!", true);
        return true;
    }

    public static Pose3d getTagPose(int tagId) {
        return TAG_FIELD_LAYOUT.getTagPose(tagId).orElse(new Pose3d());
    }
}