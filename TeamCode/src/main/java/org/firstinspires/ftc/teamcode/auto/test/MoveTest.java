package org.firstinspires.ftc.teamcode.auto.test;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;

import static org.firstinspires.ftc.teamcode.auto.core.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class MoveTest extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            move(FORWARD, 20+ROBOT_LENGTH, 1);
            grabFoundation();
            move(BACKWARD, 10, 1);
            turnByGyro(100, 0.7);
//            sleep(2000);
//            move(BACKWARD, 20, 1);
//            sleep(2000);
//            move(FORWARD, 20, 1);
//            sleep(2000);
//            move(BACKWARD, 20, 1);
//            sleep(2000);
//            move(Slide.LEFT, 20, 1);
//            sleep(2000);
//            move(Slide.RIGHT, 20, 1);
//            sleep(2000);
//            move(Slide.LEFT, 20, 1);
//            sleep(2000);
//            move(Slide.RIGHT, 20, 1);
//            sleep(2000);
//            move(FORWARD_RIGHT, 20, 1);
//            sleep(2000);
//            move(BACKWARD_LEFT, 20, 1);
//            sleep(2000);
//            move(FORWARD_LEFT, 20, 1);
//            sleep(2000);
//            move(BACKWARD_RIGHT, 20, 1);
//            sleep(2000);
        }
        // stop
    }
}
