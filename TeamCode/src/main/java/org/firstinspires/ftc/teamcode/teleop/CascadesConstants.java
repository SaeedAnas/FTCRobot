package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.auto.Constants;

public class CascadesConstants extends Tele{
    static int count = 0;
    static double distance = -3.5; //actually dont know (hight of the block)
    static double COUNTS_PER_INCH = (1440 * 0.33333) / (3.85827 * Math.PI); //COUNTER_PER_INCH is differnt
    //COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI)
    static  double encoderValue;


    public void targetPosition (boolean target){
        if(cascadeLeft.getCurrentPosition() < encoderValue || cascadeRight.getCurrentPosition() < encoderValue){
            cascadeLeft.setPower(0.5);
            cascadeRight.setPower(0.5);
        }
    }
    public void cascadeUp(int block) {
        double encoderValue = COUNTS_PER_INCH * (distance * block) + 0.3;
        count++;
        if(cascadeLeft.getCurrentPosition() < encoderValue || cascadeRight.getCurrentPosition() < encoderValue){
            cascadeLeft.setPower(0.5);
            cascadeRight.setPower(0.5);
        }
    }

    public void cascadeDown(int block) {
        double encoderValue = COUNTS_PER_INCH * (distance * block);
        count--;
        while (cascadeLeft.getCurrentPosition() > encoderValue || cascadeRight.getCurrentPosition() > encoderValue){
            cascadeLeft.setPower(-0.5);
            cascadeRight.setPower(-0.5);
        }
        cascadeLeft.setPower(0);
        cascadeRight.setPower(0);
    }

    public void cascadeRest() {
        double encoderValue = COUNTS_PER_INCH * (distance * count) + (-0.3 * count);
        if(cascadeLeft.getCurrentPosition() > encoderValue || cascadeRight.getCurrentPosition() > encoderValue){
            cascadeLeft.setPower(-0.5);
            cascadeRight.setPower(-0.5);
        }
    }


    class cascadeDown implements Runnable {
        public void run(){
            try{
                cascadeDown(1);
                Thread.sleep(500);
            }catch(Exception e){}
        }
    }

    class cascadeRest implements Runnable{
        public void run() {
            try{
                cascadeRest();
                Thread.sleep(500);
            }catch (Exception e){}
        }
    }
}

