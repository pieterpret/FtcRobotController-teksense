package org.firstinspires.ftc.teamcode.components;

public class MecanumDrive extends DriveTrain{

    private final Motor fl, fr, bl, br;

    public MecanumDrive(Motor fl, Motor fr, Motor bl, Motor br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    @Override
    public void drive(
            double strafe,
            double forward,
            double rotation,
            double correction
    ) {
        double rot = rotation + correction;

        double flp = forward + strafe + rot;
        double frp = forward - strafe - rot;
        double blp = forward - strafe + rot;
        double brp = forward + strafe - rot;

        // Normalize instead of clamp
        double max = Math.max(
                Math.abs(flp),
                Math.max(Math.abs(frp), Math.max(Math.abs(blp), Math.abs(brp)))
        );

        if (max > 1.0) {
            flp /= max;
            frp /= max;
            blp /= max;
            brp /= max;
        }

        fl.setPower(flp);
        fr.setPower(frp);
        bl.setPower(blp);
        br.setPower(brp);
    }
}
