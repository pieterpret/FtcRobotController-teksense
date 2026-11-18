package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.GamePadEx;
import org.firstinspires.ftc.teamcode.components.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.Motor;
import org.firstinspires.ftc.teamcode.components.ServoEx;

@TeleOp(name="teksense teleop")

public class mecanum_test extends LinearOpMode {

    GamePadEx driver1;
    GamePadEx driver2;
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

    private ThrowState throwState = ThrowState.IDLE;

    private boolean previousA = false;
    private int sensorTriggerTicks = 0;

    private final ElapsedTime throwTimer = new ElapsedTime();

    private static final double THROW_POWER = 1.0;
    private static final double OVERSHOOT_POWER = 0.25;
    private static final int OVERSHOOT_TICKS = 10;
    private static final double THROW_TIMEOUT = 3.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initDevices();

        waitForStart();

        while (opModeIsActive()) {
            driver1.update(gamepad1);
            driver2.update(gamepad2);
            driveTrain.drive(driver1.leftStickX, -driver1.leftStickY, driver1.rightStickX,0);
            if(driver2.leftBumper)
            {
                intake.setPower(1);
            }
            else
            {
                intake.setPower(0);
            }
            if(driver2.rightBumper)
            {
                flower.open();
            }
            else
            {
                flower.close();
            }
            boolean aPressed = driver2.a;

// Start once per button press.
            if (aPressed && !previousA && throwState == ThrowState.IDLE) {
                throwState = ThrowState.SEEKING_SENSOR;
                throwTimer.reset();
            }

            previousA = aPressed;

            switch (throwState) {
                case SEEKING_SENSOR:
                    throwArm.setPower(THROW_POWER);

                    boolean magnetDetected = !touchSensor.getState();

                    if (magnetDetected) {
                        sensorTriggerTicks = throwArm.getDistance();
                        throwState = ThrowState.OVERSHOOT;
                    }
                    break;

                case OVERSHOOT:
                    int ticksPastSensor = Math.abs(
                            throwArm.getDistance() - sensorTriggerTicks
                    );

                    if (ticksPastSensor >= OVERSHOOT_TICKS) {
                        throwArm.setPower(0);
                        throwState = ThrowState.IDLE;
                    } else {
                        throwArm.setPower(OVERSHOOT_POWER);
                    }
                    break;

                case IDLE:
                default:
                    throwArm.setPower(0);
                    break;
            }

// Safety timeout for either powered state.
            if (throwState != ThrowState.IDLE
                    && throwTimer.seconds() >= THROW_TIMEOUT) {

                throwArm.setPower(0);
                throwState = ThrowState.IDLE;
            }

            telemetry.addData("Raw sensor", touchSensor.getState());
            telemetry.addData("Magnet detected", !touchSensor.getState());
            telemetry.addData("Throw running", throwState);
            telemetry.update();
        }
    }

    void initDevices(){
        driver1 = new GamePadEx(gamepad1);
        driver2 = new GamePadEx(gamepad2);
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
        rightBack.setReversed(false);

        throwArm.setBrake(true);
    }
}
