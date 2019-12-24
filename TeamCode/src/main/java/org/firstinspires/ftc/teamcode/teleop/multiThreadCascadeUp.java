package org.firstinspires.ftc.teamcode.teleop;

public class multiThreadCascadeUp extends CascadesConstants implements Runnable {
    public void run(){
        try{
            cascadeUp(1);
            Thread.sleep(500);
        }catch(Exception e){}
    }
}
