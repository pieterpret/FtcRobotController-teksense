package org.firstinspires.ftc.teamcode.components;

public class DifferentialDrive extends DriveTrain{

    private final Motor left, right;

    public DifferentialDrive(Motor left, Motor right) {
        this.left = left;
        this.right = right;
    }

    @Override
    public void drive(
            double strafe,
            double forward,
            double rotation,
            double correction
    ) {
        double rot = rotation + correction;

        double leftPower  = forward + rot;
        double rightPower = forward - rot;

        left.setPower(clamp(leftPower));
        right.setPower(clamp(rightPower));
    }
}
