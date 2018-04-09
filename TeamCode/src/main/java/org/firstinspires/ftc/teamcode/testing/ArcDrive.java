package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.DoubleGamepad;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Utils;

@Autonomous(name = "ArcDrive", group = "Testing")
public class ArcDrive extends LinearOpMode {
    private Robot robot;

    private final double DRIVE_POWER = 0.125;

    private double leftRadius;
    private double rightRadius;
    private double leftDistance;
    private double rightDistance;

    private DoubleGamepad input;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, this);

        input = new DoubleGamepad(gamepad1, gamepad2);

        telemetry.setMsTransmissionInterval(100);

        waitForStart();

        driveICurve(24, 24);
        telemetry.addLine("press A to continue");
        telemetry.update();
        while (!input.a() && opModeIsActive()) ;
        driveICurve(-24, 24);
        telemetry.addLine("press A to continue");
        telemetry.update();
        while (!input.a() && opModeIsActive()) ;
        driveICurve(-24, -24);
        telemetry.addLine("press A to continue");
        telemetry.update();
        while (!input.a() && opModeIsActive()) ;
        driveICurve(24, -24);
        //arcTest();
    }

    private void arcTest() {
        for (int radius = 4; radius <= 32; radius += 4) {
            for (int i = 1; i <= 3; i++) {
                trial(radius, i);
            }
        }
    }

    private void trial(int radius, int trial) {
        telemetry.addData("radius", radius);
        telemetry.addData("trial", trial);
        telemetry.addLine("press A to continue");
        telemetry.update();
        while (!input.a() && opModeIsActive()) ;

        driveArc(new Angle(1, Angle.Unit.ROTATIONS, Angle.Type.HEADING, Angle.Normalization.NONE), -radius);

        telemetry.addLine("press A to continue, or X to record angle");
        telemetry.update();

        while (opModeIsActive()) {
            if (input.a()) {
                break;
            } else if (input.x()) {
                Angle gyro = robot.GYRO.getHeading();
                Angle imu = robot.IMU.getHeading();
                telemetry.addLine("turn the robot, and press Y to calculate change in angle");
                telemetry.update();
                while (!input.y() && opModeIsActive()) ;
                gyro = Angle.difference(robot.GYRO.getHeading(), gyro, Angle.Normalization.ZERO_AT_CENTER);
                imu = Angle.difference(robot.IMU.getHeading(), imu, Angle.Normalization.ZERO_AT_CENTER);
                telemetry.addData("gyro angle", gyro);
                telemetry.addData("imu angle", imu);
                telemetry.addLine("press A to continue");
                telemetry.update();
                while (!input.a() && opModeIsActive()) ;
                break;
            }
        }
    }

    private void driveICurve(double a, double b) {
        a += Utils.noise();
        b += Utils.noise();
        double k = 3;

        int aSign = a < 0 ? -1 : 1;
        a = Math.abs(a);

        int bSign = b < 0 ? -1 : 1;
        b = Math.abs(b);

        double radius = b / 3;
        double alpha = (Math.PI / 2) - (Math.atan(b / (a - (2 * b / k))) - Math.atan((2 * b / k) / Math.sqrt(a * a - 4 * a * b / k + b * b)));
        double length = Math.sqrt(a * a - 4 * a * b / k + b * b);
        Angle angle = new Angle(alpha, Angle.Unit.RADIANS, Angle.Type.HEADING, Angle.Normalization.NONE);

        telemetry.addData("radius", radius);
        telemetry.addData("angle", angle);
        telemetry.addData("length", length);
        telemetry.addLine("press A to continue");
        telemetry.update();
        while (!input.a() && opModeIsActive());

        driveArc(aSign * bSign > 0 ? angle.negate() : angle, radius * aSign);
        robot.driveDistance(length * bSign, length * bSign, DRIVE_POWER);
        while (robot.isDriving() && opModeIsActive());
        driveArc(aSign * bSign < 0 ? angle.negate() : angle, radius * aSign * -1);
    }

    /**
     * Drives in a circular arc.
     *
     * @param angle  The Angle to drive around the circle; positive is counterclockwise
     * @param radius the radius of the
     */
    private void driveArc(Angle angle, double radius) {
        telemetry.addLine("driving arc");
        telemetry.addData("unconverted angle", angle.convertUnit(Angle.Unit.DEGREES));
        telemetry.addData("unconverted radius", radius);

        angle = angle.convertUnit(Angle.Unit.RADIANS);
        angle = new Angle(angle.VALUE * angleCorrectionFactor(radius), Angle.Unit.RADIANS, Angle.Type.HEADING, Angle.Normalization.NONE);
        radius = radiusCorrected(radius);

        telemetry.addData("angle", angle.convertUnit(Angle.Unit.DEGREES));
        telemetry.addData("radius", radius);
        telemetry.addLine("press B to cancel");
        telemetry.update();

        if (angle.VALUE == 0) {
            return;
        }

        leftRadius = radius + Robot.DRIVE_WIDTH / 2;
        rightRadius = radius - Robot.DRIVE_WIDTH / 2;

        leftDistance = leftRadius * angle.VALUE * -1;
        rightDistance = rightRadius * angle.VALUE * -1;

        robot.driveDistance(leftDistance, rightDistance, DRIVE_POWER);

        while (robot.isDriving() && opModeIsActive()) {
            if (input.b()) {
                robot.stopDrive();
            }
        }
    }

    /**
     * Returns the experimentally corrected radius value to achieve the target radius. Data here:
     * https://docs.google.com/spreadsheets/d/1FP9bHNiAd-eQ0ah5vWq40zdhH0rptly7lPrAfXUuQ6c/edit?usp=sharing
     *
     * @param radius the radius
     * @return the corrected radius
     */
    private double radiusCorrected(double radius) {
        if (radius == 0) {
            return 0;
        }

        int sign = radius < 0 ? -1 : 1;
        radius = Math.abs(radius);

        if (radius < Robot.DRIVE_WIDTH) {
            radius = 0.948 * radius + 0.0922;
        } else {
            radius = 0.899 * radius + 0.286;
        }

        return radius * sign;
    }

    private double angleCorrectionFactor(double radius) {
        radius = Math.abs(radius);

        if (radius < Robot.DRIVE_WIDTH) {
            return (0.094 * radius + 382) / 360;
        } else {
            return (0.574 * radius + 375) / 360;
        }
    }
}
