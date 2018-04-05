package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.util.Point;

/**
 * A simplified interface for position sensors.
 */
public interface SimplePositionSensor {
    enum Axis {
        X, Y, Z
    }

    Point getPosition();

    void reverse(Axis axis);
}
