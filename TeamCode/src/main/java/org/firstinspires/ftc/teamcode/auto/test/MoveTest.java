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
            move(FORWARD, 20, 1);
            sleep(2000);
            move(BACKWARD, 20, 1);
            sleep(2000);
            move(FORWARD, 20, 1);
            sleep(2000);
            move(BACKWARD, 20, 1);
            sleep(2000);
            move(Strafe.LEFT, 20, 1);
            sleep(2000);
            move(Strafe.RIGHT, 20, 1);
            sleep(2000);
            move(Strafe.LEFT, 20, 1);
            sleep(2000);
            move(Strafe.RIGHT, 20, 1);
            sleep(2000);
            move(FORWARD_RIGHT, 20, 1);
            sleep(2000);
            move(BACKWARD_LEFT, 20, 1);
            sleep(2000);
            move(FORWARD_LEFT, 20, 1);
            sleep(2000);
            move(BACKWARD_RIGHT, 20, 1);
            sleep(2000);
        }
        // stop
    }
}
