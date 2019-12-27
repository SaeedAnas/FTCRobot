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
            move(FORWARD, 20, 0.5);
            sleep(sleepTime);
            move(BACKWARD, 20, 0.5);
            sleep(sleepTime);
            move(LEFT, 20, 0.5);
            sleep(sleepTime);
            move(RIGHT, 20, 0.5);
            sleep(sleepTime);
            move(FORWARD_LEFT, 20, 0.5);
            sleep(sleepTime);
            move(BACKWARD_LEFT, 20,0.5);
            sleep(sleepTime);
            move(FORWARD_RIGHT, 20, 0.5);
            sleep(sleepTime);
            move(BACKWARD_RIGHT, 20, 0.5);
            sleep(sleepTime);
        }
    }
}
