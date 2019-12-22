package org.firstinspires.ftc.teamcode.auto.test;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;

import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MoveTest extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            autoCorrectMove(FORWARD, TILE_LENGTH, 0.3);
            sleep(1000);
            autoCorrectMove(BACKWARD,TILE_LENGTH, 0.3);
            sleep(1000);
            autoCorrectMove(Strafe.LEFT,TILE_LENGTH, 0.3);
            sleep(1000);
            autoCorrectMove(Strafe.RIGHT, TILE_LENGTH, 0.3);
            sleep(1000);
            autoCorrectMove(FORWARD_LEFT,TILE_LENGTH, 0.3);
            sleep(1000);
            autoCorrectMove(BACKWARD_LEFT, TILE_LENGTH, 0.3);
            sleep(1000);
            autoCorrectMove(FORWARD_RIGHT,TILE_LENGTH, 0.3);
            sleep(1000);
            autoCorrectMove(BACKWARD_RIGHT,TILE_LENGTH, 0.3);
        }
        // stop
    }
}
