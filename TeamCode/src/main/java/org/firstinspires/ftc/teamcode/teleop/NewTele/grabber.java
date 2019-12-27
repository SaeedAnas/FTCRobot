package org.firstinspires.ftc.teamcode.teleop.NewTele;

public class grabber extends Button {
    @Override
    void run() {
        if (dpad1Left) {
            grabber.setPosition(1);
        } else if (dpad1Right) {
            grabber.setPosition(0);
        }
    }
}
