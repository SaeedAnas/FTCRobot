package org.firstinspires.ftc.teamcode.auto.test;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Calibrate extends Autonomous {
    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            calibrateMotors();
//            calibrate();
        }
        // stop
    }

}
