package org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.util.Angle;

/**
 * A simplified interface for gyro sensors.
 */
public interface SimpleGyroSensor {
    /**
     * Defines the orientation of the gyro.
     */
    enum Orientation {
        /**
         * Normal orientation of the gyro (facing up).
         */

        NORMAL(1),
        /**
         * Reversed orientation (upside-down).
         */
        REVERSE(-1);

        /**
         * Value to multiply unconverted heading value by to return heading value corrected for
         * gyro orientation.
         */
        final int SCALAR;

        Orientation(int scalar) {
            SCALAR = scalar;
        }
    }

    /**
     * Set the gyro orientation.
     * @param orientation The gyro orientation
     */
    void setOrientation(Orientation orientation);

    /**
     * Get the gyro heading.
     * @return The gyro heading.
     */
    Angle getHeading();

    /**
     * Start gyro calibration.
     */
    void calibrate();

    /**
     * Return whether the gyro is calibrating.
     * @return true if the gyro is calibrating.
     */
    boolean isCalibrating();
}