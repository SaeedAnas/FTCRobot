package org.firstinspires.ftc.robotcontroller.external.samples;

import android.app.Activity;


import org.firstinspires.ftc.robotcontroller.auto.Autonomous;
import org.firstinspires.ftc.robotcontroller.internal.*;

import static org.firstinspires.ftc.robotcontroller.internal.SingletonManager.singleton;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class CameraCapture extends Autonomous {
    private FtcRobotControllerActivity activity = singleton.getActivity();

    private void takePicture() {
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                activity.getFragment().captureStillPicture();
            }
        });
    }

    @Override
    public void runOpMode() {
        initHardware();
        if (opModeIsActive()) {
            takePicture();
        }
        // stop
    }




}
