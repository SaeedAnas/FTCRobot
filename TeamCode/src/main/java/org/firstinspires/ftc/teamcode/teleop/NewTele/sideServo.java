package org.firstinspires.ftc.teamcode.teleop.NewTele;

public class sideServo extends Button {
    @Override
    void run() {
        if (dpad1Up) {
            sideServo.setPosition(1);
        } else if (dpad1Down) {
            sideServo.setPosition(0);
        }
    }
}
