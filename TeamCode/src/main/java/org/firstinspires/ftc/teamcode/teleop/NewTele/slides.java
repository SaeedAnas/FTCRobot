package org.firstinspires.ftc.teamcode.teleop.NewTele;

public class slides extends Button {
    @Override
    void run() {
        if (dpad2Up) {
            cascadeLeft.setPower(0.5);
            cascadeRight.setPower(0.5);
        } else if (dpad2down) {
            cascadeLeft.setPower(-0.5);
            cascadeRight.setPower(-0.5);
        }
    }
}
