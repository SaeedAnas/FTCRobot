package org.firstinspires.ftc.teamcode.auto.test;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;

import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.core.Autonomous.Strafe.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class EncoderTest extends Autonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        if(opModeIsActive()) {
            int sleepTime = 10000;
            double distRatio = 1.1;
            double dist = 20*distRatio;
            autoCorrectMove(FORWARD,dist , 0.2);
            sleep(sleepTime);
            autoCorrectMove(BACKWARD, dist, 0.2);
            sleep(sleepTime);
            autoCorrectMove(LEFT, dist, 0.5);
            sleep(sleepTime);
            autoCorrectMove(RIGHT, dist, 0.5);
            sleep(sleepTime);
            autoCorrectMove(FORWARD_LEFT, dist, 0.5);
            sleep(sleepTime);
            autoCorrectMove(BACKWARD_LEFT, dist,0.5);
            sleep(sleepTime);
            move(FORWARD_RIGHT, dist, 0.5);
            sleep(sleepTime);
            move(BACKWARD_RIGHT, dist, 0.5);
            sleep(sleepTime);
        }
    }
}

