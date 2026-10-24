package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.GamePadEx;
import org.firstinspires.ftc.teamcode.components.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.Motor;

@TeleOp(name="mecanum test")
@Disabled
public class mecanum_test extends LinearOpMode {

    GamePadEx driver1;
    Motor leftFront;
    Motor rightFront;
    Motor leftBack;
    Motor rightBack;
    DriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {
        driver1 = new GamePadEx(gamepad1);
        leftFront = new Motor(hardwareMap,"leftFront");
        rightFront = new Motor(hardwareMap,"rightFront");
        leftBack = new Motor(hardwareMap,"leftBack");
        rightBack = new Motor(hardwareMap,"rightBack");
        driveTrain = new MecanumDrive(leftFront,rightFront,leftBack,rightBack);

        leftFront.setReversed(true);
        rightFront.setReversed(false);
        leftBack.setReversed(true);
        rightBack.setReversed(false);

        waitForStart();

        while (opModeIsActive()) {
            driver1.update(gamepad1);
            driveTrain.drive(driver1.leftStickX, -driver1.leftStickY, driver1.rightStickX,0);
        }
    }
}
