package org.firstinspires.ftc.teamcode.util;

/**
 * Represents a vector in 2D or 3D space.
 */
public class Vector extends Point {
    /**
     * Creates a new Vector with the specified x and y components. z is set to zero.
     * @param x the x component
     * @param y the y component
     */
    public Vector(double x, double y) {
        this(x, y, 0);
    }

    /**
     * Creates a new Vector with the specified x, y, and z components.
     * @param x the x component
     * @param y the y component
     * @param z the z component
     */
    public Vector(double x, double y, double z) {
        super(x, y, z);
    }

    /**
     * Returns the magnitude of this Vector.
     *
     * @return The magnitude of this Vector
     */
    public double magnitude() {
        return distance(ZERO, this);
    }

    /**
     * Returns the magnitude of this Vector before applying square root.
     *
     * @return The squared magnitude of this Vector
     */
    public double magnitudeSquared() {
        return distanceSquared(ZERO, this);
    }

    /**
     * Returns the Angle coterminal to this Vector. Ignores the z component.
     *
     * @return The coterminal Angle
     */
    public Angle getAngle() {
        return new Angle(getAngleValue(), Angle.Unit.RADIANS, Angle.Type.UNIT_CIRCLE, Angle.Normalization.ZERO_TO_MAX);
    }

    private double getAngleValue() {
        if (Y == 0) {
            return X < 0 ? Math.PI : 0;
        } else if (X == 0) {
            return Y < 0 ? (3 * Math.PI / 2) : (Math.PI / 2);
        } else {
            double angle = Math.atan(Y / X);
            if (X < 0) {
                angle += Math.PI;
            }
            return angle;
        }
    }
}
