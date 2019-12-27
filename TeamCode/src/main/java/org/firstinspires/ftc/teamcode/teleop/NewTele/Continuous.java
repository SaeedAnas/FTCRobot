package org.firstinspires.ftc.teamcode.teleop.NewTele;

public class Continuous extends Button {

    @Override
    void run() {
        if (leftY2 > 0) {
            blockMover.setPower(0.7);
        } else if (leftY2 < 0) {
            blockMover.setPower(-0.7);
        }
    }
}
