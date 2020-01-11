package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class VisionOpModeVuforia extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        while(opModeIsActive()) {
            printBlockPositon();
        }
        stopVision();
    }
}
