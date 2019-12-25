package org.firstinspires.ftc.teamcode.teleop.NewTele;

public class multiThreadCascadeUp extends CascadeConstants implements Runnable {
    public void run(){
        try{
            cascadeUp(1);
            Thread.sleep(500);
        }catch(Exception e){}
    }
}

