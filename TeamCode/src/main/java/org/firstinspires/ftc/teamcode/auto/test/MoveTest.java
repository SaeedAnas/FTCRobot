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
            move(FORWARD, 10, 0.5);
            telemetry.addData("Bruh", foundationLeft);
            telemetry.addData("Bruh2", foundationRight);
        }
        // stop
    }
}
