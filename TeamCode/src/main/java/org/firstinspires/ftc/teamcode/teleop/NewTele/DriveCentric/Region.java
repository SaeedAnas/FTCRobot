package org.firstinspires.ftc.teamcode.teleop.NewTele.DriveCentric;

public enum Region {
    NORTH {
        @Override
        public double calculateFrontRight(double x, double y, double rx) {
            return (y - x - rx);
        }

        @Override
        public double calculateFrontLeft(double x, double y, double rx) {
            return (y + x + rx);
        }

        @Override
        public double calculateBottomRight(double x, double y, double rx) {
            return (y + x - rx);
        }

        @Override
        public double calculateBottomLeft(double x, double y, double rx) {
            return (y - x + rx);
        }
    },
    NORTH_EAST {
        @Override
        public double calculateFrontRight(double x, double y, double rx) {
            return y - rx;
        }

        @Override
        public double calculateFrontLeft(double x, double y, double rx) {
            return x + rx;
        }

        @Override
        public double calculateBottomRight(double x, double y, double rx) {
            return x -rx;
        }

        @Override
        public double calculateBottomLeft(double x, double y, double rx) {
            return y + rx;
        }
    },
    EAST {
        @Override
        public double calculateFrontRight(double x, double y, double rx) {
            return x + y - rx;
        }

        @Override
        public double calculateFrontLeft(double x, double y, double rx) {
            return x - y + rx;
        }

        @Override
        public double calculateBottomRight(double x, double y, double rx) {
            return x - y - rx;
        }

        @Override
        public double calculateBottomLeft(double x, double y, double rx) {
            return x + y + rx;
        }
    },
    SOUTH_EAST {
        @Override
        public double calculateFrontRight(double x, double y, double rx) {
            return -x-rx;
        }

        @Override
        public double calculateFrontLeft(double x, double y, double rx) {
            return -y+rx;
        }

        @Override
        public double calculateBottomRight(double x, double y, double rx) {
            return -y-rx;
        }

        @Override
        public double calculateBottomLeft(double x, double y, double rx) {
            return -x+rx;
        }
    },
    SOUTH {
        @Override
        public double calculateFrontRight(double x, double y, double rx) {
            return (-y + x - rx);
        }

        @Override
        public double calculateFrontLeft(double x, double y, double rx) {
            return (-y - x + rx);
        }

        @Override
        public double calculateBottomRight(double x, double y, double rx) {
            return (-y - x - rx);
        }

        @Override
        public double calculateBottomLeft(double x, double y, double rx) {
            return (-y + x + rx);
        }
    },
    SOUTH_WEST {
        @Override
        public double calculateFrontRight(double x, double y, double rx) {
            return -y-rx;
        }

        @Override
        public double calculateFrontLeft(double x, double y, double rx) {
            return -x+rx;
        }

        @Override
        public double calculateBottomRight(double x, double y, double rx) {
            return -x-rx;
        }

        @Override
        public double calculateBottomLeft(double x, double y, double rx) {
            return -y+rx;
        }
    },
    WEST {
        @Override
        public double calculateFrontRight(double x, double y, double rx) {
            return -x - y - rx;
        }

        @Override
        public double calculateFrontLeft(double x, double y, double rx) {
            return -x + y + rx;
        }

        @Override
        public double calculateBottomRight(double x, double y, double rx) {
            return -x + y - rx;
        }

        @Override
        public double calculateBottomLeft(double x, double y, double rx) {
            return -x - y + rx;
        }
    },
    NORTH_WEST {
        @Override
        public double calculateFrontRight(double x, double y, double rx) {
            return x-rx;
        }

        @Override
        public double calculateFrontLeft(double x, double y, double rx) {
            return y+rx;
        }

        @Override
        public double calculateBottomRight(double x, double y, double rx) {
            return y-rx;
        }

        @Override
        public double calculateBottomLeft(double x, double y, double rx) {
            return x+rx;
        }
    };

    public abstract double calculateFrontRight(double x, double y, double rx);
    public abstract double calculateFrontLeft(double x, double y, double rx);
    public abstract double calculateBottomRight(double x, double y, double rx);
    public abstract double calculateBottomLeft(double x, double y, double rx);



    }
