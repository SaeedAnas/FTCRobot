package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.Constants.*;

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
