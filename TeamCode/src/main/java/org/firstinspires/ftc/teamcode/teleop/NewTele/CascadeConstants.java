package org.firstinspires.ftc.teamcode.teleop.NewTele;
import org.firstinspires.ftc.teamcode.teleop.Tele;

public class CascadeConstants extends ThreadButton{
    static int count = 0;
    static double distance = -3.5; //actually dont know (hight of the block)
    static double PullyCurrcum = 2;
    static double countPerRev = 1440;
    static double COUNTS_PER_INCH = countPerRev / (PullyCurrcum * Math.PI); //COUNTER_PER_INCH is differnt
    //COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI)
    static double encoderValue;
    static double max = 6;
    static double min = 0;
    static boolean mutexUp = true;
    static boolean mutexDown = true;
    static boolean mutexReset = true;


    public void targetPosition (boolean target){
        if(cascadeLeft.getCurrentPosition() < encoderValue || cascadeRight.getCurrentPosition() < encoderValue){
            cascadeLeft.setPower(0.5);
            cascadeRight.setPower(0.5);
        }
    }
    public void cascadeUp(int block) {
        double encoderValue = COUNTS_PER_INCH * (distance * block) + 0.3;
        count++;
        if(count < max) {


            while (cascadeLeft.getCurrentPosition() < encoderValue || cascadeRight.getCurrentPosition() < encoderValue) {
                cascadeLeft.setPower(0.5);
                cascadeRight.setPower(0.5);
                telemetry.addData("Status: ", "Cascade Up");
                telemetry.update();
            }
            cascadeLeft.setPower(0);
            cascadeRight.setPower(0);
        }
    }

    public void cascadeDown(int block) {
        double encoderValue = COUNTS_PER_INCH * (distance * block);
        count--;
        if (count >= min) {
            while (cascadeLeft.getCurrentPosition() > encoderValue || cascadeRight.getCurrentPosition() > encoderValue) {
                cascadeLeft.setPower(-0.5);
                cascadeRight.setPower(-0.5);
                telemetry.addData("Status: ", "Cascade Down");
                telemetry.update();
            }
            cascadeLeft.setPower(0);
            cascadeRight.setPower(0);
        }
    }

    public void cascadeReset() {
        double encoderValue = 50;
        while(cascadeLeft.getCurrentPosition() > encoderValue || cascadeRight.getCurrentPosition() > encoderValue){
            cascadeLeft.setPower(-0.5);
            cascadeRight.setPower(-0.5);
            telemetry.addData("Status: ", "Cascade Reset");
            telemetry.update();
        }
        cascadeRight.setPower(0);
        cascadeLeft.setPower(0);
    }

    @Override
    void run() {
        Thread cascadeUp = new Thread(new multiThreadCascadeUp());
        Thread cascadeDown = new Thread(new multiThreadCascadeDown());
        Thread cascadeReset = new Thread(new multiThreadCascadeReset());

        if (dpad2Up){
            if(mutexUp==true){
                cascadeUp.start();
                mutexUp = false;
            }
        }
        else if(dpad2down){
            if(mutexDown==true){
                cascadeDown.start();
                mutexDown =false;
            }
        }
        else if(dpad2right){
            if(mutexReset==true){
                cascadeReset.start();
                mutexReset=false;
            }
        }
    }
}

//    class cacadeUp implements Runnable {
//        public void run(){
//            try{
//                cascadeUp(1);
//                Thread.sleep(500);
//            }catch(Exception e){}
//        }
//    }
//
//    class cascadeDown implements Runnable {
//        public void run(){
//            try{
//                cascadeDown(1);
//                Thread.sleep(500);
//            }catch(Exception e){}
//        }
//    }
//
//    class cascadeRest implements Runnable{
//        public void run() {
//            try{
//                cascadeRest();
//                Thread.sleep(500);
//            }catch (Exception e){}
//        }
//    }

