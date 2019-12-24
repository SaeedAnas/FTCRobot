package org.firstinspires.ftc.teamcode.teleop;

class multiThreadCascadeRest extends CascadesConstants implements Runnable{
    public void run() {
        try{
            cascadeRest();
            Thread.sleep(500);
        }catch (Exception e){}
    }
}
