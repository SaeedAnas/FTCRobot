package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.Tele;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Multithreading extends Autonomous {


     @Override
     public void runOpMode() {
         initHardware();
         int count = 1;
         while (opModeIsActive()) {
             if(count == 1)
            multithreadingDemonstration();
             count++;
         }
         // stop
     }

    static boolean[] arr = new boolean[2];

    class TelemetryRunnable implements Runnable {

        @Override
        public void run() {
            for(long i = 0; i < 10000000; i++) {
                telemetry.addData("Count: ", i);
                arr[0] = true;
            }
        }
    }

    class OtherRunnable implements Runnable {
        @Override
        public void run() {
            for (long i = 10000000; i > 0; i--) {
                telemetry.addData("Count: ", i);
                arr[1] = true;
            }
        }

    }

    class Listener implements Runnable {
        @Override
        public void run() {
            while(opModeIsActive()) {
                listen();
                arr[0] = false;
                arr[1] = false;
                telemetry.update();
            }


        }

        void listen() {
            while(!(arr[0] && arr[1])) {
            }
        }
    }



    public void multithreadingDemonstration() {
        telemetry.addData("Status: ", "Starting");
        telemetry.update();
        Thread l = new Thread(new Listener());
        Thread up = new Thread(new TelemetryRunnable());
        Thread down = new Thread(new OtherRunnable());
        telemetry.addData("RIGHTBEFOREL", "AAA");
        telemetry.update();
        l.start();
        telemetry.addData("L", " Started");
        telemetry.update();
        up.start();
        telemetry.addData("up", " Started");
        telemetry.update();
        down.start();
        telemetry.addData("down", " Started");
        telemetry.update();
        telemetry.addData("Status: ", "Finished");
        telemetry.update();
    }

}

