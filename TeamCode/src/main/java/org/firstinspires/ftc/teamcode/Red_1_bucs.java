package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.Motor;
import org.firstinspires.ftc.teamcode.components.ServoEx;

@Autonomous(name = "red 1 bucs")
public class Red_1_bucs extends LinearOpMode {
    Motor leftFront;
    Motor rightFront;
    Motor leftBack;
    Motor rightBack;
    DriveTrain driveTrain;
    Motor throwArm;
    Motor intake;
    ServoEx flower;
    DigitalChannel touchSensor;

    private enum ThrowState {
        IDLE,
        SEEKING_SENSOR,
        OVERSHOOT
    }

    private Red_1_bucs.ThrowState throwState = Red_1_bucs.ThrowState.IDLE;

    private boolean previousA = false;
    private int sensorTriggerTicks = 0;

    private final ElapsedTime throwTimer = new ElapsedTime();

    private static final double THROW_POWER = 1.0;
    private static final double OVERSHOOT_POWER = 0.25;
    private static final int OVERSHOOT_TICKS = 10;
    private static final double THROW_TIMEOUT = 3.0;
    int shotCount = 0;
    void initDevices(){

        leftFront = new Motor(hardwareMap,"leftfront");
        rightFront = new Motor(hardwareMap,"rightfront");
        leftBack = new Motor(hardwareMap,"leftback");
        rightBack = new Motor(hardwareMap,"rightback");
        driveTrain = new MecanumDrive(leftFront,rightFront,leftBack,rightBack);
        throwArm = new Motor(hardwareMap,"throwArm");
        intake = new Motor(hardwareMap,"intake");
        flower = new ServoEx(hardwareMap,"flower",0,1,false);
        touchSensor = hardwareMap.get(DigitalChannel.class,"limit");

        leftFront.setReversed(true);
        rightFront.setReversed(false);
        leftBack.setReversed(true);
        rightBack.setReversed(true);

        throwArm.setBrake(true);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initDevices();
        waitForStart();
        if(opModeIsActive())
        {
            for(int i = 0; i<4;i++) {
                throwArm.setPower(0.6);
                sleep(300);
                while (touchSensor.getState()) {
                    throwArm.setPower(1);
                    telemetry.addData("limit",touchSensor.getState());
                    telemetry.update();
                }
                throwArm.setPower(0);
                sleep(400);
                while (!touchSensor.getState()) {
                    throwArm.setPower(0.2);
                    telemetry.addData("limit",touchSensor.getState());
                    telemetry.update();
                }
                throwArm.setPower(0);
                sleep(300);
            }
        }
    }
}
