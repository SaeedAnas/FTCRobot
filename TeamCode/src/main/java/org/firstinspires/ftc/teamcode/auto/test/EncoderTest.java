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
<<<<<<< HEAD
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
=======
            try {
                int sleepTime = 10000;
                double rat = 1;
                double distance = 20 * rat;
                autoCorrect(FORWARD, distance, 0.5);
                sleep(sleepTime);
                autoCorrect(BACKWARD, distance, 0.5);
                sleep(sleepTime);
                autoCorrect(LEFT, distance, 0.5);
                sleep(sleepTime);
                autoCorrect(RIGHT, distance, 0.5);
                sleep(sleepTime);
                autoCorrect(FORWARD_LEFT, distance, 0.5);
                sleep(sleepTime);
                autoCorrect(BACKWARD_LEFT, distance, 0.5);
                sleep(sleepTime);
                autoCorrect(FORWARD_RIGHT, distance, 0.5);
                sleep(sleepTime);
                autoCorrect(BACKWARD_RIGHT, distance, 0.5);
                sleep(sleepTime);
            } catch (Exception e) {
                telemetry.addData("Kendall is gay", "This code is also gay");
                telemetry.update();
            }
>>>>>>> 842c7d9bfb02ca81baf8c78463ea410686d16e6a
        }
    }
}

