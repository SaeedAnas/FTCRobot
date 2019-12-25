package org.firstinspires.ftc.teamcode.teleop.NewTele;

public class Foundation extends Button{

    @Override
    void run() {
            if (gpad1x) {
                foundationRight.setPosition(1);
                foundationLeft.setPosition(1);
            } else if (gpad1y) {
                foundationRight.setPosition(0.5);
                foundationLeft.setPosition(0.5);
            }
    }
}
