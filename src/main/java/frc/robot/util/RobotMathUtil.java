package frc.robot.util;

import edu.wpi.first.math.util.Units;

/**
 * Does Math stuff for us
 */
public final class RobotMathUtil {
    private RobotMathUtil() {
        throw new UnsupportedOperationException("RobotMathUtil is a utility class and should not be instantiated!");
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
     * Converts an array of inch values to an array of meters values. Returns a new array.
     *
     * @param inchesArray The array of inches
     * @return The array of meters
     */
    public static double[] inchesArrayToMetersArray(double[] inchesArray) {
        double[] metersArray = new double[inchesArray.length];
        for (int i = 0; i < inchesArray.length; i++) {
            metersArray[i] = Units.inchesToMeters(inchesArray[i]);
        }
        return metersArray;
    }

    /**
     * Rotates a point around the origin
     *
     * @param x X coordinate
     * @param y Y coordinate
     * @param angle Angle to rotate by, in deg
     */
    public static double[] rotatePoint(double x, double y, double angle) {
        double[] rotatedPoint = new double[2];
        rotatedPoint[0] = x * Math.cos(Math.toRadians(angle)) - y * Math.sin(Math.toRadians(angle));
        rotatedPoint[1] = x * Math.sin(Math.toRadians(angle)) + y * Math.cos(Math.toRadians(angle));
        return rotatedPoint;
    }

    public static double roundNearestHundredth(double d) {
        return Math.floor(d * 100) / 100d;
    }

    public static double squareKeepSign(double d) {
        return d * d * Math.signum(d);
    }
}