package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.Angle;

/**
 * A wrapper class for motors that simplifies motor logic by using Angles in place of encoder ticks.
 */
public class SimpleMotor implements DcMotor {
    private final DcMotor MOTOR;
    public final double TICKS_PER_MOTOR_REVOLUTION;

    public enum RotateFreelyBy {
        /**
         * Directly scales the voltage applied to the motor by the power value.
         */
        POWER,
        /**
         * Uses encoders to run the motor at the target velocity.
         */
        VELOCITY;
    }

    public SimpleMotor(DcMotor motor, double ticksPerMotorRevolution) {
        MOTOR = motor;
        setMode(RunMode.STOP_AND_RESET_ENCODER);
        setMode(RunMode.RUN_USING_ENCODER);

        TICKS_PER_MOTOR_REVOLUTION = ticksPerMotorRevolution;
    }

    /**
     * Runs the motor in a given direction.
     *
     * @param power          The motor power between -1 and 1, where positive is forwards and
     *                       negative is backwards.
     * @param rotateFreelyBy Specifies whether the the power value should be interpreted as a
     *                       raw power or as a target velocity.
     */
    public void rotateFreely(double power, RotateFreelyBy rotateFreelyBy) {
        if (rotateFreelyBy.equals(RotateFreelyBy.POWER)) {
            if (!getMode().equals(RunMode.RUN_WITHOUT_ENCODER)) {
                setMode(RunMode.RUN_WITHOUT_ENCODER);
            }
        } else if (rotateFreelyBy.equals(RotateFreelyBy.VELOCITY)) {
            if (!getMode().equals(RunMode.RUN_USING_ENCODER)) {
                setMode(RunMode.RUN_USING_ENCODER);
            }
        }
        setPower(power);
    }

    /**
     * Runs the motor towards a target Angle specified relative to the current position.
     *
     * @param angle The target Angle
     * @param power The power to apply to the motor
     */
    public void rotate(Angle angle, double power) {
        rotateAbsolute(Angle.add(angle, getRotation(), Angle.Normalization.NONE), power);
    }

    /**
     * Runs the motor towards a target Angle specified relative to the zero position.
     *
     * @param angle The target Angle
     * @param power The power to apply to the motor
     */
    public void rotateAbsolute(Angle angle, double power) {
        setTargetPosition(angleToEncoderTicks(angle));
        if (!getMode().equals(RunMode.RUN_TO_POSITION)) {
            setMode(RunMode.RUN_TO_POSITION);
        }
        setPower(power);
    }

    /**
     * Gets the Angle rotated by this motor relative to the zero position.
     *
     * @return The rotation of this motor
     */
    public Angle getRotation() {
        return encoderTicksToAngle(getCurrentPosition());
    }

    /**
     * Stops the motor.
     */
    public void stop() {
        setPower(0);
        if (getMode().equals(RunMode.RUN_TO_POSITION)) {
            setMode(RunMode.RUN_USING_ENCODER);
        }
    }

    private Angle encoderTicksToAngle(double ticks) {
        return new Angle(ticks / TICKS_PER_MOTOR_REVOLUTION, Angle.Unit.ROTATIONS, Angle.Type.HEADING, Angle.Normalization.NONE).convertUnit(Angle.Unit.DEGREES);
    }

    private int angleToEncoderTicks(Angle angle) {
        return (int) Math.round(angle.convertUnit(Angle.Unit.ROTATIONS).VALUE * TICKS_PER_MOTOR_REVOLUTION);
    }

    //All the below methods connect the DcMotor interface methods to our DcMotor instance
    //Not much to see here

    @Override
    public MotorConfigurationType getMotorType() {
        return MOTOR.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        MOTOR.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return MOTOR.getController();
    }

    @Override
    public int getPortNumber() {
        return MOTOR.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        MOTOR.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return MOTOR.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        //deprecated
    }

    @Override
    public boolean getPowerFloat() {
        //deprecated
        return false;
    }

    @Override
    public void setTargetPosition(int position) {
        MOTOR.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return MOTOR.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return MOTOR.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return MOTOR.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        MOTOR.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return MOTOR.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        MOTOR.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return MOTOR.getDirection();
    }

    @Override
    public void setPower(double power) {
        MOTOR.setPower(power);
    }

    @Override
    public double getPower() {
        return MOTOR.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return MOTOR.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return MOTOR.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return MOTOR.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return MOTOR.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        MOTOR.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        MOTOR.close();
    }
}
