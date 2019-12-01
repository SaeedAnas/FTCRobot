package org.firstinspires.ftc.robotcontroller.internal;

import android.content.Context;

public class SingletonManager {
    public static SingletonManager singleton;
    private FtcRobotControllerActivity context;

    private SingletonManager(FtcRobotControllerActivity context) {
        this.context = context;
    }

    public synchronized static SingletonManager getInstance(FtcRobotControllerActivity context) {
        if (singleton == null) {
            singleton = new SingletonManager(context);
        }
        return singleton;
    }

    public FtcRobotControllerActivity getActivity() {
        return context;
    }
}
