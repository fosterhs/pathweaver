package frc.team8051.sensors;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro extends  ADXRS450_Gyro {
    public Gyro() {
        SmartDashboard.putData("Gyro", this);
    }

    public double getHeading() {
        return -getAngle();
    }
}
