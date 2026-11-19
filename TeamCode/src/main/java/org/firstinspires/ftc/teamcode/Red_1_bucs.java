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

    private final ElapsedTime throwTimer = new ElapsedTime();

    private static final double THROW_POWER = 0.9;
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
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        leftFront.setReversed(true);
        rightFront.setReversed(false);
        leftBack.setReversed(true);
        rightBack.setReversed(true);

        throwArm.setBrake(true);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initDevices();
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        for (int shot = 0; shot < 4 && opModeIsActive(); shot++) {
            boolean completed = runThrowCycle();

            if (!completed) {
                telemetry.addData("Throw failed", shot + 1);
                telemetry.update();
                break;
            }

            sleep(400);
        }

        throwArm.setPower(0);
        driveTrain.drive(0,-0.5,0,0);
        sleep(400);
        driveTrain.drive(0,0,-0.5,0);
        sleep(400);
        driveTrain.drive(0,-0.5,0,0);
        sleep(3200);
        driveTrain.drive(0,0,0,0);
    }

    private boolean runThrowCycle() {
        throwTimer.reset();

        /*
         * Stage 1: Move until the magnet enters the sensor.
         *
         * Assumes:
         * true  = magnet not detected
         * false = magnet detected
         */
        while (opModeIsActive()
                && touchSensor.getState()
                && throwTimer.seconds() < THROW_TIMEOUT) {

            throwArm.setPower(THROW_POWER);

            telemetry.addData("Throw stage", "Seeking sensor");
            telemetry.addData("Sensor", touchSensor.getState());
            telemetry.addData("Arm ticks", throwArm.getDistance());
            telemetry.update();

            idle();
        }

        if (!opModeIsActive() || throwTimer.seconds() >= THROW_TIMEOUT) {
            throwArm.setPower(0);
            return false;
        }

        /*
         * Stage 2: Continue slowly until the magnet has completely
         * passed through the sensor's detection area.
         */
        while (opModeIsActive()
                && !touchSensor.getState()
                && throwTimer.seconds() < THROW_TIMEOUT) {

            throwArm.setPower(OVERSHOOT_POWER);
            idle();
        }

        if (!opModeIsActive() || throwTimer.seconds() >= THROW_TIMEOUT) {
            throwArm.setPower(0);
            return false;
        }

        /*
         * Stage 3: Move a fixed number of encoder ticks beyond the
         * trailing edge of the sensor.
         */
        int clearPosition = throwArm.getDistance();

        while (opModeIsActive()
                && Math.abs(throwArm.getDistance() - clearPosition)
                < OVERSHOOT_TICKS
                && throwTimer.seconds() < THROW_TIMEOUT) {

            throwArm.setPower(OVERSHOOT_POWER);

            telemetry.addData("Throw stage", "Overshooting");
            telemetry.addData(
                    "Ticks past sensor",
                    Math.abs(throwArm.getDistance() - clearPosition)
            );
            telemetry.update();

            idle();
        }

        throwArm.setPower(0);

        return opModeIsActive()
                && throwTimer.seconds() < THROW_TIMEOUT;
    }
}
