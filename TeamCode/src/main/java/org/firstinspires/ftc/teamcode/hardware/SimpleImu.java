package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Utils;

public class SimpleImu implements SimpleGyroSensor, SimplePositionSensor {
    private final BNO055IMU IMU;

    private Orientation orientation = Orientation.NORMAL;

    private int xScalar = 1;
    private int yScalar = 1;
    private int zScalar = 1;

    public SimpleImu(BNO055IMU imu) {
        IMU = imu;
        IMU.initialize(imuParameters());
    }

    private static BNO055IMU.Parameters imuParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        return parameters;
    }

    @Override
    public void setOrientation(Orientation orientation) {
        this.orientation = orientation;
    }

    @Override
    public Angle getHeading() {
        double heading = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        heading *= orientation.SCALAR;
        return new Angle(heading, Angle.Unit.DEGREES, Angle.Type.HEADING, Angle.Normalization.ZERO_TO_MAX);
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
        Position position = IMU.getPosition();
        return new Point(position.x * xScalar, position.y * yScalar, position.z * zScalar);
    }

    @Override
    public void reverse(Axis axis) {
        switch (axis) {
            case X:
                xScalar *= -1;
                return;
            case Y:
                yScalar *= -1;
                return;
            case Z:
                zScalar *= -1;
                return;
        }
    }

    @Override
    public String toString() {
        return Utils.format("heading %s position %s", getHeading(), getPosition());
    }
}
