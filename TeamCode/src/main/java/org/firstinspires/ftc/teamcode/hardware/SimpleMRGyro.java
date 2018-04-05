package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import org.firstinspires.ftc.teamcode.util.Angle;

public class SimpleMRGyro implements SimpleGyroSensor {
    private final ModernRoboticsI2cGyro GYRO;
    private Orientation orientation = Orientation.NORMAL;

    public SimpleMRGyro(ModernRoboticsI2cGyro gyro) {
        GYRO = gyro;
    }

    @Override
    public void setOrientation(Orientation orientation) {
        this.orientation = orientation;
    }

    @Override
    public Angle getHeading() {
        return new Angle(GYRO.getHeading() * orientation.SCALAR, Angle.Unit.DEGREES,
                Angle.Type.HEADING, Angle.Normalization.ZERO_TO_MAX);
    }

    @Override
    public void calibrate() {
        GYRO.calibrate();
    }

    @Override
    public boolean isCalibrating() {
        return GYRO.isCalibrating();
    }

    @Override
    public String toString() {
        return getHeading().toString();
    }
}
