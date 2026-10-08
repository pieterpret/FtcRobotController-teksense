package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.components.DifferentialDrive;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.Motor;
import org.firstinspires.ftc.teamcode.components.MotorEx;
import org.firstinspires.ftc.teamcode.components.ServoEx;

@Autonomous(name = "red 1")
public class Auto_1 extends LinearOpMode {
    //definitions for motors and servos
    Motor leftMotor;
    Motor rightMotor;
    MotorEx intake;
    MotorEx flywheel;
    ServoEx crServo;
    DriveTrain driveTrain;

    //PIDF constants for flywheel
    private static final double kP = 5.0;
    private static final double kI = 0.1;
    private static final double kD = 0.0;
    private static final double kF = 12.0;
    private static final double FLYWHEEL_VELOCITY = 1300;
    private static final double INTAKE_POWER = 0.8;
    private static final double SPIN_UP_SECONDS = 1.5;

    private final ElapsedTime flywheelTimer = new ElapsedTime();
    private boolean flywheelEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = new Motor(hardwareMap, "leftDrive");
        rightMotor = new Motor(hardwareMap, "rightDrive");
        intake = new MotorEx(hardwareMap, "intakeMotor");
        flywheel = new MotorEx(hardwareMap, "flywheel");
        crServo = new ServoEx(hardwareMap, "crServo", 0, 1);
        driveTrain = new DifferentialDrive(leftMotor,rightMotor);

        leftMotor.setReversed(false);
        rightMotor.setReversed(true);
        //PIDF setup for flywheel
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(kP, kI, kD, kF));

        crServo.setPosition(0.5);

        waitForStart();

        if(opModeIsActive()) {
            move(-360, 0.5);
            setFlywheel(2,10);
            turn(150, -0.5);
            move(-1500,1);
            turn(130, 0.5);
            move(-2500, 0.5);
            turn(360,0.5);
        }
    }
    //move and turn functions
    void move(int distanceTicks, double maxPower) {
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        int target = Math.abs(distanceTicks);
        double direction = distanceTicks >= 0 ? 1.0 : -1.0;

        double kP = 0.003;
        double minimumPower = 0.12;
        int toleranceTicks = 10;

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < 8.0) {
            int leftTicks = Math.abs(leftMotor.getDistance());
            int rightTicks = Math.abs(rightMotor.getDistance());

            int leftError = target - leftTicks;
            int rightError = target - rightTicks;

            boolean leftFinished = leftError <= toleranceTicks;
            boolean rightFinished = rightError <= toleranceTicks;

            if (leftFinished && rightFinished) {
                break;
            }

            double leftCommand = calculateMovePower(
                    leftError, leftFinished, maxPower, minimumPower
            );

            double rightCommand = calculateMovePower(
                    rightError, rightFinished, maxPower, minimumPower
            );

            leftCommand *= direction;
            rightCommand *= direction;

            /*
             * DifferentialDrive uses:
             *
             * left  = forward + rotation
             * right = forward - rotation
             *
             * These equations produce the desired individual wheel powers.
             */
            double forward = (leftCommand + rightCommand) / 2.0;
            double rotation = (leftCommand - rightCommand) / 2.0;

            driveTrain.drive(0, forward, rotation, 0);

            telemetry.addData("Target", target);
            telemetry.addData("Left", "%d, error %d", leftTicks, leftError);
            telemetry.addData("Right", "%d, error %d", rightTicks, rightError);
            telemetry.addData("Power", "L %.2f, R %.2f",
                    leftCommand, rightCommand);
            telemetry.addData("Time", timer.seconds());
            telemetry.update();

            idle();
        }

        driveTrain.drive(0, 0, 0, 0);

        telemetry.addData(
                "Move ended",
                timer.seconds() >= 8.0 ? "Timeout" : "Target reached"
        );
        telemetry.update();

        sleep(500);
    }

    double calculateMovePower(
            int error,
            boolean finished,
            double maxPower,
            double minimumPower
    ) {
        if (finished) {
            return 0;
        }

        double proportionalPower = 0.003 * error;

        return Range.clip(
                proportionalPower,
                minimumPower,
                Math.abs(maxPower)
        );
    }

    void turn(int targetTicks, double power) {
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        ElapsedTime timer = new ElapsedTime();
        int target = Math.abs(targetTicks);

        while (
                opModeIsActive()
                        && timer.seconds() < 5.0
                        && (
                        Math.abs(leftMotor.getDistance()) < target
                                || Math.abs(rightMotor.getDistance()) < target
                )
        ) {
            driveTrain.drive(0, 0, power, 0);

            telemetry.addData("Target ticks", target);
            telemetry.addData("Left ticks", leftMotor.getDistance());
            telemetry.addData("Right ticks", rightMotor.getDistance());
            telemetry.addData("Time", timer.seconds());
            telemetry.update();

            idle();
        }

        driveTrain.drive(0, 0, 0, 0);

        telemetry.addData("Turn ended", timer.seconds() >= 5.0
                ? "Timeout"
                : "Target reached");
        telemetry.update();

        sleep(500);
    }

    void setFlywheel(double spinUpSeconds, double feedSeconds) {
        ElapsedTime timer = new ElapsedTime();

        // Start only the flywheel.
        flywheel.setVelocity(FLYWHEEL_VELOCITY);
        intake.setPower(0);
        crServo.setPosition(0.5);

        // Wait for it to reach operating speed.
        timer.reset();

        while (opModeIsActive() && timer.seconds() < spinUpSeconds) {
            telemetry.addData("Stage", "Spinning up");
            telemetry.addData("Target velocity", FLYWHEEL_VELOCITY);
            telemetry.addData("Actual velocity", flywheel.getVelocity());
            telemetry.update();

            idle();
        }

        if (!opModeIsActive()) {
            stopFlywheel();
            return;
        }

        // Flywheel, intake, and feeder now operate together.
        intake.setPower(INTAKE_POWER);
        crServo.open();

        timer.reset();

        while (opModeIsActive() && timer.seconds() < feedSeconds) {
            telemetry.addData("Stage", "Feeding");
            telemetry.addData("Target velocity", FLYWHEEL_VELOCITY);
            telemetry.addData("Actual velocity", flywheel.getVelocity());
            telemetry.update();

            idle();
        }

        stopFlywheel();
    }

    void stopFlywheel() {
        flywheel.setPower(0);
        intake.setPower(0);
        crServo.setPosition(0.5);
    }
}
