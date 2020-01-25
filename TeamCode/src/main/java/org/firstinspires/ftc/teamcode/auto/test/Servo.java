package org.firstinspires.ftc.teamcode.auto.test;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Servo extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        while (opModeIsActive()) {
            scanMode();
            sleep(1000);
            off();
            sleep(1000);

        }
        // stop
    }

}
