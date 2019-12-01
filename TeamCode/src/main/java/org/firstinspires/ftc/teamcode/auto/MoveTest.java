package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.Constants.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MoveTest extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            move(FORWARD, TILE_LENGTH, 0.3);
            sleep(1000);
            move(BACKWARD,TILE_LENGTH, 0.3);
            sleep(1000);
        }
        // stop
    }
}
