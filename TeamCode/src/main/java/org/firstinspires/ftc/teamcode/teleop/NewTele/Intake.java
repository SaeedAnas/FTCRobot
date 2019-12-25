package org.firstinspires.ftc.teamcode.teleop.NewTele;

public class Intake extends Button {
    @Override
    void run() {
        if (gpad1a) {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        } else if (gpad1b){
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }
    }
}
