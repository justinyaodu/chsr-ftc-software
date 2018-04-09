package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Calculates the current robot position using the distances traveled by the
 * left and right wheels.
 */
public class EncoderPositionSensor implements SimpleGyroSensor, SimplePositionSensor {
    private final SimpleMotor LEFT_MOTOR;
    private final SimpleMotor RIGHT_MOTOR;

    //x and y position of robot
    private double x = 0;
    private double y = 0;

    //robot angle on the unit circle
    private double angle = Math.PI / 2;

    //last recorded encoder values for motors
    private Angle lastLeftAngle;
    private Angle lastRightAngle;

    public EncoderPositionSensor(SimpleMotor leftMotor, SimpleMotor rightMotor) {
        LEFT_MOTOR = leftMotor;
        RIGHT_MOTOR = rightMotor;

        lastLeftAngle = LEFT_MOTOR.getRotation();
        lastRightAngle = RIGHT_MOTOR.getRotation();
    }

    @Override
    public void setOrientation(Orientation orientation) {
        //do nothing, this should never be called
    }

    @Override
    public Angle getHeading() {
        update();
        return new Angle(angle, Angle.Unit.RADIANS, Angle.Type.UNIT_CIRCLE, Angle.Normalization.ZERO_TO_MAX)
                .convertType(Angle.Type.HEADING).convertUnit(Angle.Unit.DEGREES);
    }

    @Override
    public void calibrate() {
        //do nothing
    }

    @Override
    public boolean isCalibrating() {
        return false;
    }

    @Override
    public Point getPosition() {
        update();
        return new Point(x, y);
    }

    @Override
    public void reverse(Axis axis) {
        //do nothing, this should never be called
    }

    //recalculates the current robot position based on the change in motor position
    private void update() {
        Angle leftAngle = LEFT_MOTOR.getRotation();
        Angle rightAngle = RIGHT_MOTOR.getRotation();

        double leftRotation = Angle.difference(leftAngle, lastLeftAngle, Angle.Normalization.NONE)
                .convertUnit(Angle.Unit.ROTATIONS).VALUE;
        double rightRotation = Angle.difference(rightAngle, lastRightAngle, Angle.Normalization.NONE)
                .convertUnit(Angle.Unit.ROTATIONS).VALUE;

        double leftInches = leftRotation * Robot.DRIVE_GEAR_RATIO * Robot.WHEEL_CIRCUMFERENCE_IN;
        double rightInches = rightRotation * Robot.DRIVE_GEAR_RATIO * Robot.WHEEL_CIRCUMFERENCE_IN;

        move(leftInches, rightInches);

        lastLeftAngle = leftAngle;
        lastRightAngle = rightAngle;
    }

    //recalculates robot position based on distance driven by left and right wheels
    private void move(double leftInches, double rightInches) {
        //a hacky but effective way to prevent division by zero
        leftInches += Utils.noise();
        rightInches += Utils.noise();

        double arcRadius = (Robot.DRIVE_WIDTH / 2) * (rightInches + leftInches) / (rightInches - leftInches);
        double arcLength = (leftInches + rightInches) / 2;
        double arcAngle = arcLength / arcRadius;

        double translateAngle = arcAngle / 2;
        double translateDistance = Math.sqrt(2 * arcRadius * arcRadius * (1 - Math.cos(arcAngle)));

        x += translateDistance * Math.cos(angle + translateAngle);
        y += translateDistance * Math.sin(angle + translateAngle);
        angle += arcAngle;
    }

    @Override
    public String toString() {
        return Utils.format("heading %s position %s", getHeading(), getPosition());
    }
}
