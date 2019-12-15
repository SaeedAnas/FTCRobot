package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.Constants.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Servo extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            for(int i = 0; i <= 1; i+=0.1) {
                moveFoundation(i,i);
                telemetry.addData("ServoVal: ", i);
                telemetry.update();
                sleep(1000);
            }
        }
        // stop
    }

}
