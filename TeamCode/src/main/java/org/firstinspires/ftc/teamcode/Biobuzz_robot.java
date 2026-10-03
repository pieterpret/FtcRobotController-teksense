package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//api abstraction classes
import org.firstinspires.ftc.teamcode.components.DifferentialDrive;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.GamePadEx;
import org.firstinspires.ftc.teamcode.components.GyroEx;
import org.firstinspires.ftc.teamcode.components.Motor;
import org.firstinspires.ftc.teamcode.components.MotorEx;
import org.firstinspires.ftc.teamcode.components.ServoEx;

@TeleOp(name="teksense teleop")
public class Biobuzz_robot extends LinearOpMode {
    GamePadEx driver1;
    GamePadEx driver2;
    Motor leftMotor;
    Motor rightMotor;
    MotorEx flywheel;
    MotorEx intake;
    ServoEx feeder;
    DriveTrain driveTrain;

    // PIDF constants for flywheel
    private static final double kP = 6.0;
    private static final double kI = 0.5;
    private static final double kD = 0.0;
    private static final double kF = 12.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initControls();
        waitForStart();

        while (opModeIsActive())
        {
            driver1.update(gamepad1);
            driver2.update(gamepad2);
            driveTrain.drive(0, driver1.leftStickY, -driver1.rightStickX,0);

            if(driver2.leftBumper)
            {
                intake.setVelocity(1020);
            }
            else
            {
                intake.setVelocity(0);
            }

            if(driver2.rightBumper){
                flywheel.setVelocity(1200);
            }
            else
            {
                flywheel.setVelocity(0);
            }

            if(flywheel.getVelocity() >= 1000)
            {
                feeder.setPosition(0);
            }
            else
            {
                feeder.setPosition(0.5);
            }
        }
    }

    void initControls(){
        driver1 = new GamePadEx(gamepad1);
        driver2 = new GamePadEx(gamepad2);

        leftMotor = new Motor(hardwareMap,"leftDrive");
        rightMotor = new Motor(hardwareMap,"rightDrive");
        flywheel = new MotorEx(hardwareMap,"flywheel");
        intake = new MotorEx(hardwareMap,"intakeMotor");
        feeder = new ServoEx(hardwareMap,"crServo",-1,1);

        leftMotor.setReversed(true);
        rightMotor.setReversed(false);
        feeder.setPosition(0.5);

        driveTrain = new DifferentialDrive(leftMotor,rightMotor);
        // Flywheel PIDF setup
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF));
    }
}
