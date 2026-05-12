package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

/*
 *                  MotorEx Helper Class
 *
 * Purpose:
 *   Extends the Motor base class to explicitly use DcMotorEx features.
 *   Adds advanced control methods such as run-to-position, velocity control,
 *   and target position checks.
 *
 * Benefits:
 *   - Makes advanced motor features explicit and easy to use
 *   - Provides convenience methods for moving by ticks or checking tolerance
 *   - Keeps base Motor class lightweight while exposing Ex-only functionality
 *   - Also allow pidf tuning if needed for more precise control
 *
 * Usage:
 *   MotorEx liftMotor = new MotorEx(hardwareMap, "lift");
 *   liftMotor.runToPosition(2000, 0.8);
 *   while (!liftMotor.isAtTarget(20)) {
 *       telemetry.addData("Lift pos", liftMotor.getDistance());
 *       telemetry.update();
 *   }
 *
 * Notes:
 *   Ensure the hardware configuration uses a DcMotorEx device.
 *   If not, instantiating MotorEx will throw an error.
 */

public class MotorEx extends Motor{
    private DcMotorEx motorEx;
    private static final int SAMPLE_SIZE = 20;
    private double[] errorSamples = new double[SAMPLE_SIZE];
    private int sampleIndex = 0;
    ElapsedTime timer;
    public double speed;

    public MotorEx(HardwareMap hw, String name)
    {
        super(hw, name);
        motorEx = hw.get(DcMotorEx.class, name);
    }

    public void runToPosition(int targetTicks, double power)
    {
        motorEx.setTargetPosition(targetTicks);
        motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorEx.setPower(Math.abs(clamp(power)));
    }

    public void moveBy(int deltaTicks, double power)
    {
        runToPosition(motorEx.getCurrentPosition() + deltaTicks, power);
    }

    public boolean isAtTarget(int toleranceTicks)
    {
        return Math.abs(
                motorEx.getTargetPosition() - motorEx.getCurrentPosition()
        ) <= toleranceTicks;
    }

    // If you need extended features, check motorEx
    public void setVelocity(double ticksPerSecond)
    {

        motorEx.setVelocity(ticksPerSecond);
    }

    public double getVelocity() {

        return motorEx.getVelocity();
    }

    public DcMotorEx getMotorEx()
    {
        return motorEx;
    }

    // PIDF helpers
    public PIDFCoefficients getPIDFCoefficients(DcMotorEx.RunMode mode) {
        return motorEx.getPIDFCoefficients(mode);
    }

    public void setPIDFCoefficients(DcMotorEx.RunMode mode, PIDFCoefficients coeffs) {
        motorEx.setPIDFCoefficients(mode, coeffs);
    }

    public void autoTunePIDF(double targetVelocity, double time ) {
        PIDFCoefficients coeffs = new PIDFCoefficients(5.0, 0.0, 0.0, 10.0);
        motorEx.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);

        motorEx.setVelocity(targetVelocity);

        while (timer.seconds() < time)
        {
            speed = motorEx.getVelocity();
        }

        double actual = motorEx.getVelocity();
        double error = targetVelocity - actual;

        // Adjust P
        if (Math.abs(error) > 100) coeffs.p += 1.0;

        // Adjust I if persistent offset remains
        if (Math.abs(error) > 50 && Math.abs(error) < 100) coeffs.i += 0.1;

        // Adjust D if oscillations are detected (you’d measure velocity variance)
        if (detectOscillation()) coeffs.d += 0.05;

        // Adjust F if steady-state error persists
        if (error > 0) coeffs.f += 0.5;

        motorEx.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
    }



    public void recordError(double error) {
        errorSamples[sampleIndex] = error;
        sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;
    }

    private boolean detectOscillation() {
        int signChanges = 0;
        for (int i = 1; i < SAMPLE_SIZE; i++) {
            if (Math.signum(errorSamples[i]) != Math.signum(errorSamples[i-1])) {
                signChanges++;
            }
        }
        // If error flips sign more than half the time, call it oscillation
        return signChanges > SAMPLE_SIZE / 2;
    }


    //private functions
    private double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }


}
