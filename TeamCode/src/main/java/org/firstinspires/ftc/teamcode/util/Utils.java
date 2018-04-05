package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;
import java.util.Locale;

/**
 * A collection of general utility methods.
 */
public class Utils {
    /**
     * Linearly interpolates between two values.
     *
     * @param a the start value
     * @param b the end value
     * @param t the interpolation value, with 0 corresponding to a and 1 corresponding to b
     * @return the value between a and b interpolated by t, clamped between a and b
     */
    public static double lerp(double a, double b, double t) {
        return clamp((b - a) * t + a, Math.min(a, b), Math.max(a, b));
    }

    /**
     * Returns the average of a sequence of double values.
     *
     * @param values The values to average
     * @return The average
     */
    public static double average(double... values) {
        double sum = 0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.length;
    }

    /**
     * Clamps a value within a range.
     *
     * @param d The initial value
     * @param a One endpoint of the acceptable range
     * @param b The other endpoint of the acceptable range
     * @return d clamped between a and b
     */
    public static double clamp(double d, double a, double b) {
        return Math.min(Math.max(a, b), Math.max(Math.min(a, b), d));
    }

    /**
     * Generates a miniscule non-zero noise value. Useful for preventing division by zero.
     *
     * @return a non-zero value between -1x10e-8 and 1x10e-8
     */
    public static double noise() {
        double noise;
        do {
            noise = (Math.random() - 0.5) * 10e-8;
        } while (noise == 0);
        return noise;
    }

    /**
     * Formats a String using String.format, but quietly uses the Locale.ENGLISH locale so that
     * Android Studio doesn't complain. Also handles exceptions so that the program does not crash.
     *
     * @param format The format string
     * @param args   The parameters
     * @return The formatted string
     */
    public static String format(String format, Object... args) {
        try {
            return String.format(Locale.ENGLISH, format, args);
        } catch (Exception e) {
            return "STRING FORMAT FAILED: " + Arrays.toString(e.getStackTrace());
        }
    }
}

