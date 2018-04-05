package org.firstinspires.ftc.teamcode.util;

/**
 * Represents a point in 2D or 3D space.
 */
public class Point {
    public final double X;
    public final double Y;
    public final double Z;
    public static final Point ZERO = new Point(0, 0, 0);

    /**
     * Creates a new Point with x and y coordinates. z is set to zero.
     * @param x The x coordinate
     * @param y The y coordinate
     */
    public Point(double x, double y) {
        this(x, y, 0);
    }

    /**
     * Creates a new Point with x, y, and z coordinates.
     * @param x The x coordinate
     * @param y The y coordinate
     * @param z The z coordinate
     */
    public Point(double x, double y, double z) {
        X = x;
        Y = y;
        Z = z;
    }

    /**
     * Returns a Vector from this Point to another.
     *
     * @param other
     * @return
     */
    public Vector vectorTo(Point other) {
        return new Vector(other.X - X, other.Y - Y, other.Z - Z);
    }

    /**
     * Returns the square of the distance between two Points. Useful when only
     * comparing distances to each other, saving a square root operation.
     *
     * @param a the first Point
     * @param b the second Point
     * @return the square of the distance between the Points
     */
    public static double distanceSquared(Point a, Point b) {
        return Math.pow(b.X - a.X, 2) + Math.pow(b.Y - a.Y, 2) + Math.pow(b.Z - a.Z, 2);
    }

    /**
     * Returns the distance between two Points.
     *
     * @param a the first Point
     * @param b the second Point
     * @return the distance between the Points
     */
    public static double distance(Point a, Point b) {
        return Math.sqrt(distanceSquared(a, b));
    }

    /**
     * Returns a String representation of this Point.
     * @return A String with this Point's x, y, and z coordinates
     */
    @Override
    public String toString() {
        return Utils.format("(%.4f, %.4f, %.4f)", X, Y, Z);
    }
}
