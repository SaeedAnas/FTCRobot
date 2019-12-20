package org.firstinspires.ftc.teamcode.auto;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Calibrate extends Autonomous {
    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            calibrateMotors();
//            calibrate();
        }
        // stop
    }

}
