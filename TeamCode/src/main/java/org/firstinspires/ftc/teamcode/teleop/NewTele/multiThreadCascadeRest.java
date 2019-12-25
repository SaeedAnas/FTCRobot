package org.firstinspires.ftc.teamcode.teleop.NewTele;

class multiThreadCascadeRest extends CascadeConstants implements Runnable{
    public void run() {
        try{
            cascadeRest();
            Thread.sleep(500);
        }catch (Exception e){}
    }
}
