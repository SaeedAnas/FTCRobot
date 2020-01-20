package org.firstinspires.ftc.teamcode.teleop.NewTele.DriveCentric;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import static org.firstinspires.ftc.teamcode.teleop.NewTele.DriveCentric.Region.*;
import org.firstinspires.ftc.teamcode.teleop.NewTele.DriveCentric.DriveCentricTeleOp;

public class ImuService implements Runnable {
    private BNO055IMU imu;
    private double startingAngle;
    private double currentAngle;
    private double actualAngle;

    public ImuService(BNO055IMU imu) {
        this.imu = imu;
        resetService();
    }

    public void resetService() {
        startingAngle = getGyroYAngle();
        currentAngle = startingAngle;

    }
    /**
     * gets the Y value from the imu
     * @return Y value of the imu
     */
    private double getGyroYAngle() {
        actualAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        return actualAngle;
    }

    private double getAdjustedGyroYAngle() {
        return adjustedAngle(startingAngle, getGyroYAngle());
    }

    /**
     *
     * @param zeroReference the angle the robot was at during the beginning of the turn
     * @param currentAngle the current angle of the robot
     * @return the difference between the currentAngle and zeroReference
     */
    private double adjustedAngle(double zeroReference, double currentAngle) {
        double adjusted = currentAngle - zeroReference;
        if (adjusted < -179) {
            adjusted += 360;
        } else if (adjusted > 180) {
            adjusted -= 360;
        }
        return adjusted;
    }

    private Region calculateRegion() {
        if (currentAngle > -22.5 && currentAngle < 22.5) {
            return NORTH;
        }
        else if (currentAngle > 22.5 && currentAngle < 67.5) {
            return NORTH_EAST;
        }
        else if (currentAngle > 67.5 && currentAngle < 112.5) {
            return EAST;
        }
        else if (currentAngle > 112.5 && currentAngle < 157.5) {
            return SOUTH_EAST;
        }
        else if (currentAngle > 157.5 || currentAngle < -157.5) {
            return SOUTH;
        }
        else if (currentAngle > -157.5 && currentAngle < -112.5) {
            return SOUTH_WEST;
        }
        else if (currentAngle > -112.5 && currentAngle < -67.5) {
            return WEST;
        }
        else if (currentAngle > -67.5 && currentAngle < -22.5) {
            return NORTH_WEST;
        }
        else return NORTH;
    }

    public Region getRegion() {
        return calculateRegion();
    }

    public double getStartingAngle() {
        return startingAngle;
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public double getActualAngle() {
        return actualAngle;
    }

    @Override
    public void run() {
        while(!Thread.interrupted())
            currentAngle = getAdjustedGyroYAngle();
    }
}
