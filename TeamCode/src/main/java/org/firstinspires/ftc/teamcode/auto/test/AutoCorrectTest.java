package org.firstinspires.ftc.teamcode.auto.test;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;

import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutoCorrectTest extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        while (opModeIsActive()) {
            autoCorrectMove(FORWARD, TILE_LENGTH, 0);
            sleep(1000);
            autoCorrectMove(BACKWARD, TILE_LENGTH, 0);
            sleep(1000);
        }

        // stop
    }

}
