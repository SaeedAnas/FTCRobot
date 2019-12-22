package org.firstinspires.ftc.robotcontroller.internal;

import android.content.Context;
import android.view.View;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SingletonManager {
    public static SingletonManager singleton;
    private FtcRobotControllerActivity context;
    private ImageView stream;

    private SingletonManager(FtcRobotControllerActivity context, View stream) {
        this.context = context;
        this.stream = (ImageView) stream;
    }

    public synchronized static SingletonManager getInstance(FtcRobotControllerActivity context, View stream) {
        if (singleton == null) {
            singleton = new SingletonManager(context,stream);
        }
        return singleton;
    }

    public FtcRobotControllerActivity getActivity() {
        return context;
    }
    public ImageView getStream() {
        return stream;
    }


}
