package org.firstinspires.ftc.teamcode;
//main api classes
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

@TeleOp(name = "tele op test")
public class FTC_base_class extends LinearOpMode
{
    private GamePadEx driver1;
    GyroEx imu;
    Motor leftMotor;
    Motor rightMotor;
    MotorEx flywheel;
    ServoEx gate;
    DriveTrain driveTrain;


    boolean FlywheelStop;


    @Override
    public void runOpMode(){
        driver1 = new GamePadEx(gamepad1);
        leftMotor = new Motor(hardwareMap,"leftMotor");
        rightMotor = new Motor(hardwareMap,"rightMotor");
        imu = new GyroEx(hardwareMap,"IMU",RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        driveTrain = new DifferentialDrive(leftMotor,rightMotor);

        leftMotor.setReversed(true);
        rightMotor.setReversed(false);

        waitForStart();

        while ((opModeIsActive()))
        {
            driver1.update(gamepad1);
            driveTrain.drive(0, driver1.leftStickY, driver1.leftStickX, imu.getCorrection(0,0.2));

        }
    }


}
