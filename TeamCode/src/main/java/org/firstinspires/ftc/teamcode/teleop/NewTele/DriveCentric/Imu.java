package org.firstinspires.ftc.teamcode.teleop.NewTele.DriveCentric;

import com.qualcomm.hardware.bosch.BNO055IMU;

public class Imu {
    public static Imu Imu;
    private static BNO055IMU localImu;

    private Imu(BNO055IMU imu) {
        this.localImu = imu;
    }

    public synchronized static Imu getInstance(BNO055IMU localImu) {
        if (Imu == null) {
            Imu = new Imu(localImu);
        }
        return Imu;
    }

    public static BNO055IMU getImu() {
        return localImu;
    }


}
