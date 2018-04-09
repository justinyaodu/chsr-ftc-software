package org.firstinspires.ftc.teamcode.util;

public class Angle {
    /**
     * Defines how an Angle is measured.
     */
    public enum Type {
        /**
         * Measures starting from the positive x-axis moving counterclockwise, like in trigonometry.
         */
        UNIT_CIRCLE,
        /**
         * Measures starting from the positive y-axis moving clockwise, like a heading.
         */
        HEADING;
    }

    /**
     * Defines the unit that an Angle is measured in.
     */
    public enum Unit {
        RADIANS(2 * Math.PI, "rad"),
        DEGREES(360, "deg"),
        ROTATIONS(1, "rot");

        public final double ROTATION;
        private final String NAME;

        Unit(double rotation, String name) {
            ROTATION = rotation;
            NAME = name;
        }

        @Override
        public String toString() {
            return NAME;
        }
    }

    /**
     * Defines how an Angle is normalized.
     */
    public enum Normalization {
        /**
         * No normalization. Useful for measuring angular speed or distance.
         */
        NONE,
        /**
         * Normalizes the angle between zero and one rotation. This is typically how gyros return
         * their values.
         */
        ZERO_TO_MAX,
        /**
         * Normalizes the angle between half a negative rotation and half a positive rotation.
         * Useful for calculating differences between angles.
         */
        ZERO_AT_CENTER;
    }

    public final Unit UNIT;
    public final Type TYPE;
    public final Normalization NORMALIZATION;
    public final double VALUE;

    /**
     * Adds two Angles using the given Normalization. Uses the units of the first Angle. Uses the
     * raw angle value; Type information is ignored.
     *
     * @param a The first Angle
     * @param b The second Angle
     * @return The sum of the Angles
     */
    public static Angle add(Angle a, Angle b, Normalization normalization) {
        return new Angle(a.VALUE + b.convertUnit(a.UNIT).VALUE, a.UNIT, a.TYPE, normalization);
    }

    /**
     * Returns the difference between two Angles (a - b) using the given Normalization. Uses the
     * units of the first Angle. Uses the raw angle value; Type information is ignored.
     *
     * @param a The first Angle
     * @param b The second Angle
     * @return The angular difference of a and b
     */
    public static Angle difference(Angle a, Angle b, Normalization normalization) {
        return new Angle(a.VALUE - b.convertUnit(a.UNIT).VALUE, a.UNIT, a.TYPE, normalization);
    }

    /**
     * Creates a new Angle.
     *
     * @param value         The raw angle measurement
     * @param unit          The Unit
     * @param type          The Type
     * @param normalization The Normalization
     */
    public Angle(double value, Unit unit, Type type, Normalization normalization) {
        UNIT = unit;
        NORMALIZATION = normalization;
        TYPE = type;
        VALUE = normalize(value, unit, NORMALIZATION);
    }

    /**
     * Get this angle converted to a different Unit.
     *
     * @param unit The Unit to convert to
     * @return The converted Angle
     */
    public Angle convertUnit(Unit unit) {
        return new Angle(VALUE / UNIT.ROTATION * unit.ROTATION, unit, TYPE, NORMALIZATION);
    }

    /**
     * Get this angle position converted to a different Type.
     *
     * @param type The Type to convert to
     * @return The converted Angle
     */
    public Angle convertType(Type type) {
        if (type == TYPE) {
            return this;
        }
        return new Angle(UNIT.ROTATION / 4 - VALUE, UNIT, type, NORMALIZATION);
    }

    /**
     * Get this angle renormalized using a different Normalization.
     *
     * @param normalization The Normalization to use
     * @return The renormalized Angle
     */
    public Angle convertNormalization(Normalization normalization) {
        return new Angle(VALUE, UNIT, TYPE, normalization);
    }

    /**
     * Negates an Angle.
     * @return This Angle negated
     */
    public Angle negate() {
        return new Angle(-VALUE, UNIT, TYPE, NORMALIZATION);
    }

    /**
     * Return the sine of this Angle.
     *
     * @return The sine of this angle
     */
    public double sin() {
        return Math.sin(mathValue());
    }

    /**
     * Return the cosine of this Angle.
     *
     * @return The cosine of this Angle
     */
    public double cos() {
        return Math.cos(mathValue());
    }

    /**
     * Return the tangent of this Angle.
     *
     * @return The tangent of this Angle
     */
    public double tan() {
        return Math.tan(mathValue());
    }

    /**
     * Returns a Vector representing the ray that this angle is coterminal with.
     *
     * @return The Angle converted to a Vector
     */
    public Vector toVector() {
        return new Vector(cos(), sin());
    }

    private double mathValue() {
        return convertType(Type.UNIT_CIRCLE).convertUnit(Unit.RADIANS).VALUE;
    }

    private static double normalize(double value, Unit unit, Normalization normalization) {
        switch (normalization) {
            case ZERO_TO_MAX:
                return normalizeZeroToMax(value, unit);
            case ZERO_AT_CENTER:
                value = normalizeZeroToMax(value, unit);
                if (value > unit.ROTATION / 2) {
                    value -= unit.ROTATION;
                }
                return value;
            default:
                return value;
        }
    }

    private static double normalizeZeroToMax(double value, Unit unit) {
        return ((value % unit.ROTATION) + unit.ROTATION) % unit.ROTATION;
    }

    @Override
    public String toString() {
        return Utils.format("%.4f %s %s %s", VALUE, UNIT, TYPE, NORMALIZATION);
    }
}
