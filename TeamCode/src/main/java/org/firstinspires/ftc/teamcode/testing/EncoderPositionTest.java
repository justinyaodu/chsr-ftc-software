package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.EncoderPositionSensor;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.SimpleMotor;
import org.firstinspires.ftc.teamcode.input.DoubleGamepad;
import org.firstinspires.ftc.teamcode.util.Interpolator;

@TeleOp(name = "EncoderPositionTest", group = "Testing")
public class EncoderPositionTest extends OpMode {
    private Robot robot;
    private DoubleGamepad gamepad;

    private Interpolator leftInterpolator = new Interpolator(2);
    private Interpolator rightInterpolator = new Interpolator(2);

    private EncoderPositionSensor encoderPositionSensor;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, this);
        gamepad = new DoubleGamepad(gamepad1, gamepad2);
        telemetry.setMsTransmissionInterval(100);

        encoderPositionSensor = new EncoderPositionSensor(robot.LEFT_MOTOR, robot.RIGHT_MOTOR);
    }

    @Override
    public void loop() {
        robot.driveDirection(
                leftInterpolator.value(-gamepad.leftStickY()),
                rightInterpolator.value(-gamepad.rightStickY()),
                SimpleMotor.RotateFreelyBy.POWER);

        telemetry.addData("encoder", encoderPositionSensor);
        telemetry.addData("imu", robot.IMU);
        telemetry.addData("gyro", robot.GYRO);
    }
}