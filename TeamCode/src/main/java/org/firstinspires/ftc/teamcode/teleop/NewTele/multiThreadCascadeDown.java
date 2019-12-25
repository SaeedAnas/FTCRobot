package org.firstinspires.ftc.teamcode.teleop.NewTele;

public class multiThreadCascadeDown extends CascadeConstants implements Runnable {
    public void run(){
        try{
            mutexDown = false;
            cascadeDown(1);
            Thread.sleep(500);
            mutexDown= true;
        }catch(Exception e){}
    }
}
