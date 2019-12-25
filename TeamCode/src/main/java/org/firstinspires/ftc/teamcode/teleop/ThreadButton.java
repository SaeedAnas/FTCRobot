package org.firstinspires.ftc.teamcode.teleop;

public abstract class ThreadButton  extends Tele implements Runnable {
    boolean button;
    boolean isAvailable;

    abstract void update();

    static void begin(ThreadButton button ) {
        button.isAvailable = false;
    }

    static void finish(ThreadButton button) {
        button.isAvailable = true;
    }
}
