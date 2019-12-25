package org.firstinspires.ftc.teamcode.teleop;

public abstract class ArmButton extends ThreadButton {
    public static int min = 0;
    public static int max;
    public static int COUNTS_PER_INCH;
    abstract void move();

    void run(ArmButton button) {
        if (button.isAvailable) {
            if(button.button) {
                begin(button);
                button.move();
                finish(button);
            }
        }
    }

    class UpButton extends ArmButton {

        UpButton() {
            this.button = gamepad1.dpad_up;
            this.isAvailable = true;
        }

        @Override
        void update() {
            this.button = gamepad1.dpad_up;
        }

        @Override
        void move() {

        }

        @Override
        public void run() {
            run(this);
        }
    }
    class DownButton extends ArmButton {



        @Override
        void update() {

        }

        @Override
        public void run() {
            run(this);
        }

        @Override
        void move() {

        }
    }
    class ResetButton extends ArmButton {


        @Override
        void update() {

        }

        @Override
        public void run() {
            run(this);
        }

        @Override
        void move() {

        }
    }
}
