package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class GyroEx {
    private IMU imu;

    public GyroEx(HardwareMap hw, String name,
                  RevHubOrientationOnRobot.LogoFacingDirection logo,
                  RevHubOrientationOnRobot.UsbFacingDirection usb) {
        imu = hw.get(IMU.class, name);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb)));
        resetHeading();
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    // Proportional correction helper
    public double getCorrection(double targetHeading, double gain) {
        double error = targetHeading - getHeading();
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        return error * gain;
    }
}

