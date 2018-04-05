package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Angle;

public class Robot {
    /**
     * The robot's wheel circumferences in inches
     */
    //90 mm wheel diameter -> convert to inches -> find circumference
    public static final double WHEEL_CIRCUMFERENCE_IN = 90 * 0.0393701 * Math.PI;

    /**
     * The gear ratio between the drive motor and wheel shaft
     */
    //number of teeth on gear on drive motor shaft / number of teeth on gear on wheel shaft
    public static final double DRIVE_GEAR_RATIO = 45.0 / 30.0;

    /**
     * The distance between wheels on opposite sides of the robot
     */
    public static final double DRIVE_WIDTH = 12.625;

    /**
     * The left drive motor
     */
    public final SimpleMotor LEFT_MOTOR;

    /**
     * The right drive motor
     */
    public final SimpleMotor RIGHT_MOTOR;

    /**
     * The Modern Robotics I2C gyro
     */
    public final SimpleMRGyro GYRO;

    /**
     * The Expansion Hub IMU
     */
    public final SimpleImu IMU;

    /**
     * The number of encoder ticks per revolution of the drive motors
     */
    private static final double TICKS_PER_MOTOR_REVOLUTION = 1120;

    /**
     * Creates a new Robot from the hardware map.
     *
     * @param hardwareMap The HardwareMap to use for hardware initialization
     * @param opMode      An OpMode reference so that we can call opModeIsActive() while sensors
     *                    are calibrating, etc. and so that we can log data using telemetry.
     */
    public Robot(HardwareMap hardwareMap, OpMode opMode) {
        opMode.telemetry.addLine("initializing motors");
        opMode.telemetry.update();

        LEFT_MOTOR = new SimpleMotor(hardwareMap.dcMotor.get("leftMotor"), TICKS_PER_MOTOR_REVOLUTION);
        RIGHT_MOTOR = new SimpleMotor(hardwareMap.dcMotor.get("rightMotor"), TICKS_PER_MOTOR_REVOLUTION);
        LEFT_MOTOR.setDirection(DcMotorSimple.Direction.FORWARD);
        RIGHT_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);

        opMode.telemetry.addLine("initializing and calibrating gyros");
        opMode.telemetry.update();

        //create a SimpleMRGyro from the Modern Robotics gyro
        GYRO = new SimpleMRGyro((ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro"));

        //create a SimpleImu from the Expansion Hub IMU
        IMU = new SimpleImu(hardwareMap.get(BNO055IMU.class, "imu"));
        IMU.setOrientation(SimpleGyroSensor.Orientation.REVERSE);

        //start gyro calibrate and wait, checking to make sure opmode is still active
        GYRO.calibrate();
        while (GYRO.isCalibrating()) {
            if (opMode instanceof LinearOpMode && !((LinearOpMode)opMode).opModeIsActive()) {
                opMode.telemetry.addLine("calibration aborted, opModeIsActive returned false");
                opMode.telemetry.update();
                return;
            }
        }

        opMode.telemetry.addLine("hardware initialization complete");
        opMode.telemetry.update();
    }

    /**
     * Set the power applied to the drive motors.
     *
     * @param leftPower      The left motor power
     * @param rightPower     The right motor power
     * @param rotateFreelyBy Whether the motor power should be interpreted as a raw power or a
     *                       target velocity
     */
    public void driveDirection(double leftPower, double rightPower, SimpleMotor.RotateFreelyBy rotateFreelyBy) {
        LEFT_MOTOR.rotateFreely(leftPower, rotateFreelyBy);
        RIGHT_MOTOR.rotateFreely(rightPower, rotateFreelyBy);
    }

    /**
     * Drives the left and right sets of wheels a specified distance. The power will be scaled so
     * that both motors reach their target positions at approximately the same time.
     *
     * @param leftInches  The distance the left wheels should travel.
     * @param rightInches The distance the right wheels should travel.
     * @param power       The maximum power to be applied to the motors.
     */
    public void driveDistance(double leftInches, double rightInches, double power) {
        if (leftInches == 0 && rightInches == 0) {
            return;
        }

        double leftRotations = leftInches / WHEEL_CIRCUMFERENCE_IN / DRIVE_GEAR_RATIO;
        double rightRotations = rightInches / WHEEL_CIRCUMFERENCE_IN / DRIVE_GEAR_RATIO;

        double leftPower = power * leftInches / Math.max(Math.abs(leftInches), Math.abs(rightInches));
        double rightPower = power * rightInches / Math.max(Math.abs(leftInches), Math.abs(rightInches));

        LEFT_MOTOR.rotate(new Angle(leftRotations, Angle.Unit.ROTATIONS, Angle.Type.HEADING, Angle.Normalization.NONE), leftPower);
        RIGHT_MOTOR.rotate(new Angle(rightRotations, Angle.Unit.ROTATIONS, Angle.Type.HEADING, Angle.Normalization.NONE), rightPower);
    }

    /**
     * Stop the robot.
     */
    public void stopDrive() {
        LEFT_MOTOR.stop();
        RIGHT_MOTOR.stop();
    }

    /**
     * Returns whether the robot is driving to a target position.
     *
     * @return true if the robot is still driving towards a target position.
     */
    public boolean isDriving() {
        return LEFT_MOTOR.isBusy() || RIGHT_MOTOR.isBusy();
    }
}
