package org.firstinspires.ftc.teamcode.teleop.NewTele;

class multiThreadCascadeReset extends CascadeConstants implements Runnable{
    public void run() {
        try{
            mutexReset = false;
            cascadeReset();
            Thread.sleep(500);
            mutexReset = true;
        }catch (Exception e){}
    }
}
