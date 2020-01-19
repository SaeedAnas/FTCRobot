package org.firstinspires.ftc.teamcode.auto.test;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;

import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class imuTest extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            turnByGyro(0.9, 30);
            sleep(1000);
            turnByGyro(0.9, 15);
            sleep(1000);
            turnByGyro(0.9, -45);
            sleep(1000 );
            turnByGyro(0.9, -90);
        }
        // stop
    }
}