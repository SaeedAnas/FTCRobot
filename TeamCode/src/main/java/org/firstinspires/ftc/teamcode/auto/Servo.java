package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.Autonomous.Direction.*;
import static org.firstinspires.ftc.teamcode.auto.Constants.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Servo extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            telemetry.addData("Value Left: ", 0.5);
            telemetry.addData("Value Right: ", 0.5);
            telemetry.update();
            moveFoundation(0.5, 0.5);
            sleep(1000);
            telemetry.addData("Value Left: ", 0);
            telemetry.addData("Value Right: ", 0);
            telemetry.update();
            moveFoundation(0, 0);
            sleep(1000);
            telemetry.addData("Value Left: ", 1);
            telemetry.addData("Value Right: ", 1);
            telemetry.update();
            moveFoundation(1, 1);
            sleep(1000);
        }
        // stop
    }
}
