/*
Copyright 2026 FIRST Tech Challenge Team 21309

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
@Disabled
public class Teleop extends LinearOpMode {
    //state machine enums. DO NOT TOUCH!!!!!
    enum ShootState {
        IDLE,
        SPINUP,
        ARM_MOVE,
        ARM_HOME,
        FEED,
        COMPLETE
    }

    ShootState shootState = ShootState.IDLE;
    ElapsedTime timer = new ElapsedTime();
    int shotsRequested = 0;
    int shotsFired = 0;

    enum ShootTriggerState {
        IDLE,
        GATE_OPEN,
        SHOOTING,
        GATE_CLOSE
    }

    ShootTriggerState shootTriggerState = ShootTriggerState.IDLE;
    ElapsedTime shootTimer = new ElapsedTime();

    enum GateState { IDLE, MOVING, DONE }
    GateState gateState = GateState.IDLE;

    enum FeedState { IDLE, OPEN, CLOSE }
    FeedState feedState = FeedState.IDLE;
    ElapsedTime feedTimer = new ElapsedTime();


    // Drive train
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, gateArm, advArm;
    private DcMotorEx intakeRoller, flywheel;
    private Servo feedServo;

    private TouchSensor advArmTouch;
    
    // PIDF constants for flywheel
    private static final double kP = 5.0;
    private static final double kI = 0.1;
    private static final double kD = 0.0;
    private static final double kF = 12.0;

    // === Shooting constants ===
    private static final double FLYWHEEL_SHOOT_VELOCITY = 2000; // ticks/sec
    private static final long FLYWHEEL_SPINUP_MS = 1000;
    private static final int SHOOTCOUNT = 2;

    private static final double FEED_OPEN = 1.0;
    private static final double FEED_CLOSED = 0.0;
    private static final long FEED_PULSE_MS = 900;
    private static final long INTAKE_VELOCITY = 1500;
    
    double longitude,lateral,yaw;
    
    @Override
    public void runOpMode() {
        // === Hardware mapping ===
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");
        intakeRoller = hardwareMap.get(DcMotorEx.class, "intakeRoller");
        advArm =hardwareMap.get(DcMotor.class, "advArm");
        gateArm = hardwareMap.get(DcMotor.class, "gateArm");
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        advArmTouch = hardwareMap.get(TouchSensor.class, "advArmTouch");
        feedServo = hardwareMap.get(Servo.class, "feedServo");
        
        intakeRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeRoller.setDirection(DcMotor.Direction.REVERSE);
        
        // Flywheel PIDF setup
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
            new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF));
        
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        feedServo.setPosition(0);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            getStickPos();
            drive(-longitude,lateral,yaw);
            if(gamepad2.b)
            {
                startIntake();

            }
            else
            {
                stopIntake();

            }
            shootMan();
            updateShoot();
            updateGate();   // keep gate state machine alive
            updateFeed();   // keep feed servo state machine alive

            telemetry.addData("Shoot State", shootState);
            telemetry.addData("Trigger State", shootTriggerState);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
    
    void getStickPos()
    {
        longitude = gamepad1.left_stick_y;
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;
    }
    
    void drive(double y,double x, double yaw)
    {
        double frontLeftPower = y + x + yaw;
        double frontRightPower = y - x - yaw;
        double backRightPower = y + x - yaw;
        double backleftPower = y - x + yaw;
        
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backleftPower);
        backRightDrive.setPower(backRightPower);
    
    }

    //shoot functions

    void shootMan()
    {
        if (gamepad2.a && shootTriggerState == ShootTriggerState.IDLE) {
            startGateMove(90);
            shootTriggerState = ShootTriggerState.GATE_OPEN;
            shootTimer.reset();
        }

        switch (shootTriggerState) {
            case GATE_OPEN:
                if (shootTimer.milliseconds() > 500) {
                    startShoot(SHOOTCOUNT); // non-blocking shoot sequence
                    shootTriggerState = ShootTriggerState.SHOOTING;
                }
                break;
            case SHOOTING:
                // let startShoot() handle its own state machine
                // when it reports complete, advance:
                if (shootState == ShootState.COMPLETE) {
                    startGateMove(-90);
                    shootTriggerState = ShootTriggerState.GATE_CLOSE;
                    shootTimer.reset();
                }
                break;
            case GATE_CLOSE:
                if (shootTimer.milliseconds() > 500) {
                    shootTriggerState = ShootTriggerState.IDLE;
                }
                break;

            case IDLE:
            default:
                // do nothing
                break;
        }

    }

    private void startFlywheel() {
        flywheel.setVelocity(FLYWHEEL_SHOOT_VELOCITY);
    }

    private void stopFlywheel() {
        flywheel.setVelocity(0);
    }

    void startFeed() {
        feedServo.setPosition(FEED_OPEN);
        feedState = FeedState.OPEN;
        feedTimer.reset();
    }

    void updateFeed() {
        switch (feedState) {
            case OPEN:
                if (feedTimer.milliseconds() > FEED_PULSE_MS) {
                    feedServo.setPosition(FEED_CLOSED);
                    feedState = FeedState.CLOSE;
                    feedTimer.reset();
                }
                break;
            case CLOSE:
                // finished pulse
                feedState = FeedState.IDLE;
                break;
            case IDLE:
            default:
                break;
        }
    }


    private void startIntake()
    {
        intakeRoller.setVelocity(INTAKE_VELOCITY);
    }

    private void stopIntake()
    {
        intakeRoller.setVelocity(0);
    }

   void startGateMove(int target) {
        gateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gateArm.setTargetPosition(target);
        gateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gateArm.setPower(1);
        gateState = GateState.MOVING;
    }

    void updateGate() {
        if (gateState == GateState.MOVING) {
            if (!gateArm.isBusy()) {
                gateArm.setPower(0.0);
                gateState = GateState.DONE;
            }
        }
    }


    public void startShoot(int shots) {
        shotsRequested = shots;
        shotsFired = 0;
        startFlywheel();
        shootState = ShootState.SPINUP;
        timer.reset();
    }

    public void updateShoot() {
        switch (shootState) {
            case SPINUP:
                if (timer.milliseconds() > FLYWHEEL_SPINUP_MS) {
                    shootState = ShootState.ARM_MOVE;
                    timer.reset();
                }
                break;
            case ARM_MOVE:
                if (advArm.getCurrentPosition() != 5) {
                    advArm.setPower(-0.5);
                }
                if (timer.milliseconds() > 500) {
                    shootState = ShootState.ARM_HOME;
                    timer.reset();
                }
                break;
            case ARM_HOME:
                if (!advArmTouch.isPressed()) {
                    advArm.setPower(1);
                } else {
                    advArm.setPower(0);
                    shootState = ShootState.FEED;
                    timer.reset();
                }
                break;
            case FEED:
                startFeed();
                shotsFired++;
                telemetry.addData("Shot", shotsFired);
                telemetry.update();

                if (shotsFired < shotsRequested) {
                    shootState = ShootState.ARM_MOVE; // loop back for next shot
                    timer.reset();
                } else {
                    shootState = ShootState.COMPLETE;
                    stopFlywheel();
                }
                break;
            case COMPLETE:
                shootState = ShootState.IDLE;
                break;
            case IDLE:
            default:
                // do nothing
                break;
        }
    }
}
