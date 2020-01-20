package org.firstinspires.ftc.teamcode.auto.test;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Servo extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        while (opModeIsActive()) {
            moveServo(0.7);
            sleep(2000);
            moveServo(-0.7);
            sleep(2000);
            moveServo(0.7);
            sleep(1500);
            moveServo(-0.7);
            sleep(1500);
        }
        // stop
    }

}
