package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
                    Motor Helper Class

    Purpose:
    Provides a constructor_style wrapper around FTC DcMotor object.
    Encapsulates basic motor control (power, Encoder distance).

    Benefits:
    - Abstracts hardwareMap calls into a reusable component
    - Simplifies OpMode code by exposing clean methods
    - Converts encoder ticks into real-world distance units

    Usage:
    Motor leftMotor = new Motor(hardwareMap,"left_drive");
    leftMotor.setPower(0.5);
    int tick = leftMotor.getDistance();
    double cm = leftMotor.getDistanceCm(10.0,1120);

    Notes:
    Always call resetEncoder() before measuring distance if needed
 */

public class Motor {
    private DcMotor motor;     // base type

    // Constructor-style setup
    public Motor(HardwareMap hw, String name) {
        // Always grab as DcMotor first
        motor = hw.get(DcMotor.class, name);
    }

    // Basic power control
    public void setPower(double power) {
        motor.setPower(power);
    }

    public int getDistance() {
        return motor.getCurrentPosition();
    }

    // Optional: convert ticks to distance (in cm)
    public double getDistanceCm(double wheelDiameterCm, int ticksPerRev) {
        double circumference = Math.PI * wheelDiameterCm;
        double revolutions = (double) motor.getCurrentPosition() / ticksPerRev;
        return revolutions * circumference;
    }

    public void setBrake(boolean brake) {
        motor.setZeroPowerBehavior(
                brake ? DcMotor.ZeroPowerBehavior.BRAKE
                        : DcMotor.ZeroPowerBehavior.FLOAT
        );
    }

    public void setReversed(boolean reversed) {
        motor.setDirection(
                reversed ? DcMotor.Direction.REVERSE
                        : DcMotor.Direction.FORWARD
        );
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public DcMotor getMotor() {
        return motor;
    }

}
