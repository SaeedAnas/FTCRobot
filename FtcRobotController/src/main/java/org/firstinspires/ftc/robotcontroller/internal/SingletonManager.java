package org.firstinspires.ftc.robotcontroller.internal;

import android.content.Context;
import android.view.View;
import android.widget.ImageView;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SingletonManager {
    public static SingletonManager singleton;
    private FtcRobotControllerActivity context;
    private ImageView stream;
    private TextView mat;

    private SingletonManager(FtcRobotControllerActivity context, View stream, View mat) {
        this.context = context;
        this.stream = (ImageView) stream;
        this.mat = (TextView) mat;
    }

    public synchronized static SingletonManager getInstance(FtcRobotControllerActivity context, View stream, View mat) {
        if (singleton == null) {
            singleton = new SingletonManager(context,stream,mat);
        }
        return singleton;
    }

    public FtcRobotControllerActivity getActivity() {
        return context;
    }
    public ImageView getStream() {
        return stream;
    }
    public TextView getMatView() { return mat;}


}
