package frc.team8051.sensors;

import com.analog.adis16448.frc.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {
    private final ADIS16448_IMU imu = new ADIS16448_IMU();
    public Gyro() {
        SmartDashboard.putData("imu", imu);
    }
    public double getHeading() {
        return -imu.getAngle();
    }

    public void reset() {
        imu.reset();
    }
}
