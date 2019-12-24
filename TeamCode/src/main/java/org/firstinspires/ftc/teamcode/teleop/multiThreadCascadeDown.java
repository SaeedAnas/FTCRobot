package org.firstinspires.ftc.teamcode.teleop;

public class multiThreadCascadeDown extends CascadesConstants implements Runnable {
    public void run(){
        try{
            cascadeDown(1);
            Thread.sleep(500);
        }catch(Exception e){}
    }
}
