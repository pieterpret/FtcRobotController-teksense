package org.firstinspires.ftc.teamcode.components;

/**
 * DriveTrain – abstract base class for robot drive systems.
 *
 * Purpose:
 * Defines a common interface for all drivetrain implementations
 * (e.g. differential, mecanum, omni).
 *
 * Benefits:
 * - Keeps drivetrain logic out of the OpMode
 * - Allows different drivetrain models to be swapped easily
 * - Centralizes motion mixing and control behavior
 *
 * Usage:
 *   DriveTrain drive;
 *   drive = new DifferentialDrive(leftMotor, rightMotor);
 *   drive.drive(
 *       0,
 *       driver1.leftStickY,
 *       driver1.leftStickX,
 *       imu.getCorrection(0, 0.2)
 *   );
 */


public abstract class DriveTrain {
    public abstract void drive(
            double strafe,
            double forward,
            double rotation,
            double correction
    );

    protected double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }
}
