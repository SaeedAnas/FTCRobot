package org.firstinspires.ftc.teamcode.teleop.NewTele;

public class MechanumMovement extends Button {
    @Override
    void run() {
        move();
    }

    private void move() {
        double powerStrafe = 0.5;
        double powerStraight = 0.8;
        double powerRotate = 0.5;
        // frontLeft
        if (leftX1 < -0.3 && leftY1 < -0.3) {
            strafeLeftFront(powerStrafe);
        }
        // frontRight
        else if (leftX1 > 0.3 && leftY1 < -0.3) {
            strafeRightFront(powerStrafe);
        }
        //bottomLeft
        else if (leftX1 < -0.3 && leftY1 > 0.3) {
            strafeLeftBack(powerStrafe);
        }
        // bottomRight
        else if (leftX1 > 0.3 && leftY1 > 0.3) {
            strafeRightBack(powerStrafe);
        }
        // rotateRight
        else if (leftX1 > 0.9) {
            rotateRight(powerRotate);
        }
        // rotateLeft
        else if (leftX1 < -0.9) {
            rotateLeft(powerRotate);
        }
        // forward
        else if (leftY1 < -0.9) {
            forward(powerStraight);
        }
        // backward
        else if (leftY1 > 0.9) {
            backward(powerStraight);
        }
        // side right
        else if (rightX1 > 0.9) {
            slideRight(powerRotate);
        }
        // slide left
        else if (rightX1 < -0.9) {
            slideLeft(powerRotate);
        } else {
            stopRobot();
        }

    }


    private void stopRobot() {
        topRight.setPower(0);
        topLeft.setPower(0);
        bottomRight.setPower(0);
        bottomLeft.setPower(0);
    }


    private void forward(double power) {
        topRight.setPower(power);
        topLeft.setPower(power);
        bottomRight.setPower(power);
        bottomLeft.setPower(power);
    }

    private void backward(double power) {
        topRight.setPower(-power);
        topLeft.setPower(-power);
        bottomRight.setPower(-power);
        bottomLeft.setPower(-power);
    }

    private void slideLeft(double power) {
        topRight.setPower(power);
        topLeft.setPower(-power);
        bottomRight.setPower(-power);
        bottomLeft.setPower(power);
    }

    private void slideRight(double power) {
        topRight.setPower(-power);
        topLeft.setPower(power);
        bottomRight.setPower(power);
        bottomLeft.setPower(-power);
    }

    private void rotateRight(double power) {
        topRight.setPower(-power);
        topLeft.setPower(power);
        bottomRight.setPower(-power);
        bottomLeft.setPower(power);
    }

    private void rotateLeft(double power) {
        topRight.setPower(power);
        topLeft.setPower(-power);
        bottomRight.setPower(power);
        bottomLeft.setPower(-power);
    }

    private void strafeRightFront(double power) {
        topLeft.setPower(power);
        bottomRight.setPower(power);
    }

    private void strafeRightBack(double power) {
        topRight.setPower(-power);
        bottomLeft.setPower(-power);
    }

    private void strafeLeftFront(double power) {
        topRight.setPower(power);
        bottomLeft.setPower(power);
    }

    private void strafeLeftBack(double power) {

        topLeft.setPower(-power);
        bottomRight.setPower(-power);
    }
}
