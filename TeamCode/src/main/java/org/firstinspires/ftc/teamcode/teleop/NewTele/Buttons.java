package org.firstinspires.ftc.teamcode.teleop.NewTele;

public class Buttons extends TeleOp{
    protected static boolean gpad1x, gpad1y, gpad2a, gpad2b, gpad2x, gpad2y, gpad2rightBumper, gpad2leftBumper, gpad1rightBumper, gpad1leftBumper, dpad2Up, dpad2down, dpad2right;
    protected static double leftX1, leftY1, rightX1, rightY1, leftX2, leftY2, rightX2, rightY2, gpad1leftTrigger, gpad1rightTrigger, gpad2leftTrigger, gpad2rightTrigger;
    protected void update() {
        gpad1x = gamepad1.x;
        gpad1y = gamepad1.y;
        gpad2a = gamepad2.a;
        gpad2b = gamepad1.b;
        gpad2x = gamepad2.x;
        gpad2y = gamepad2.y;
        gpad1leftTrigger = gamepad1.left_trigger;
        gpad1rightTrigger = gamepad1.right_trigger;
        gpad2leftTrigger = gamepad2.left_trigger;
        gpad2rightTrigger = gamepad2.right_trigger;
        gpad1leftBumper = gamepad1.left_bumper;
        gpad1rightBumper = gamepad1.right_bumper;
        gpad2rightBumper = gamepad2.right_bumper;
        gpad2leftBumper = gamepad2.left_bumper;
        leftX1 = gamepad1.left_stick_x;
        leftY1 = gamepad1.left_stick_y;
        rightX1 = gamepad1.right_stick_x;
        rightY1 = gamepad1.right_stick_y;
        leftX2 = gamepad2.left_stick_x;
        leftY2 = gamepad2.left_stick_y;
        rightX2 = gamepad2.right_stick_x;
        rightY2 = gamepad2.right_stick_y;
        dpad2Up = gamepad2.dpad_up;
        dpad2down = gamepad2.dpad_down;
        dpad2right = gamepad2.dpad_right;
    }
}
