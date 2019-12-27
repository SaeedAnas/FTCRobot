package org.firstinspires.ftc.teamcode.teleop.NewTele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Buttons extends TeleOp{
    public static OpMode opmode;
    protected static boolean gpad1x, gpad1y, gpad1a, gpad1b, gpad2a, gpad2b, gpad2x, gpad2y, gpad2rightBumper, gpad2leftBumper, gpad1rightBumper, gpad1leftBumper, dpad2Up, dpad2down, dpad2right, dpad1Up, dpad1Down, dpad1Right, dpad1Left;
    protected static double leftX1, leftY1, rightX1, rightY1, leftX2, leftY2, rightX2, rightY2, gpad1leftTrigger, gpad1rightTrigger, gpad2leftTrigger, gpad2rightTrigger;

    Buttons() {
        gpad1x = gpad1y= gpad1a= gpad1b= gpad2a= gpad2b= gpad2x= gpad2y= gpad2rightBumper= gpad2leftBumper= gpad1rightBumper= gpad1leftBumper= dpad2Up= dpad2down= dpad2right= dpad1Up= dpad1Down=dpad1Right=dpad1Left = false;
        leftX1=leftY1= rightX1= rightY1=leftX2= leftY2= rightX2=rightY2= gpad1leftTrigger=gpad1rightTrigger= gpad2leftTrigger= gpad2rightTrigger = 0;
    }
    protected void update() {
        gpad1x = opmode.gamepad1.x;
        gpad1y = opmode.gamepad1.y;
        gpad1a = opmode.gamepad1.a;
        gpad2b = opmode.gamepad2.b;
        gpad2a = opmode.gamepad2.a;
        gpad2b = opmode.gamepad1.b;
        gpad2x = opmode.gamepad2.x;
        gpad2y = opmode.gamepad2.y;
        gpad1leftTrigger = opmode.gamepad1.left_trigger;
        gpad1rightTrigger =opmode.gamepad1.right_trigger;
        gpad2leftTrigger = opmode.gamepad2.left_trigger;
        gpad2rightTrigger = opmode.gamepad2.right_trigger;
        gpad1leftBumper = opmode.gamepad1.left_bumper;
        gpad1rightBumper = opmode.gamepad1.right_bumper;
        gpad2rightBumper = opmode.gamepad2.right_bumper;
        gpad2leftBumper = opmode.gamepad2.left_bumper;
        leftX1 = opmode.gamepad1.left_stick_x;
        leftY1 = opmode.gamepad1.left_stick_y;
        rightX1 = opmode.gamepad1.right_stick_x;
        rightY1 = opmode.gamepad1.right_stick_y;
        leftX2 = opmode.gamepad2.left_stick_x;
        leftY2 = opmode.gamepad2.left_stick_y;
        rightX2 =opmode. gamepad2.right_stick_x;
        rightY2 =opmode.gamepad2.right_stick_y;
        dpad2Up = opmode.gamepad2.dpad_up;
        dpad2down = opmode.gamepad2.dpad_down;
        dpad2right = opmode.gamepad2.dpad_right;
        dpad1Up = opmode.gamepad1.dpad_up;
        dpad1Down = opmode.gamepad1.dpad_down;
        dpad1Left = opmode.gamepad1.dpad_left;
        dpad1Right = opmode.gamepad1.dpad_right;
    }
}
