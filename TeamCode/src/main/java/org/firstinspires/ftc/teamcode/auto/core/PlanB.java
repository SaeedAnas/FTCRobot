package org.firstinspires.ftc.teamcode.auto.core;

import org.firstinspires.ftc.teamcode.auto.core.Autonomous;

import static org.firstinspires.ftc.teamcode.auto.core.Constants.*;
import static org.firstinspires.ftc.teamcode.auto.core.Direction.FORWARD;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class PlanB extends Autonomous {

    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            planB();
        }
        // stop
    }

    private void planB() {
        try {
            sleep(20000);
            move(FORWARD, TILE_LENGTH, DRIVE_SPEED);
            brake();
        } catch (Exception e) {
            telemetry.addData("WERE GENUINELY FUCKED","F");
            telemetry.update();
        }
    }

}
