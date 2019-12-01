package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.Constants.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class TurnTest extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
                turnByGyro(0.3, -90);
                sleep(1000);
                turnByGyro(0.3, 90);
        }
        // stop
    }
}
