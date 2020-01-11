package org.firstinspires.ftc.teamcode.auto.test;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class FoundationTest extends Autonomous {
    @Override
    public void runOpMode() {
        initHardware();
        while(opModeIsActive()){
            grabFoundation();
            sleep(1000);
            releaseFoundation();
            sleep(1000);
        }
    }
}
