package frc.team8051.sensors;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

public class DrivebaseEncoder{
    private final double whd = 6.0 / 12.0; // wheel diameter in feet
    private final double ppr = 20.0; // pulse per revolution
    private final double GEAR_RATIO = 10.75;
    private final double distancePerPulse = (whd * Math.PI) / ppr / GEAR_RATIO;

    private Encoder rightEncoder;
    private Encoder leftEncoder;

    public DrivebaseEncoder() {
        rightEncoder = new Encoder(6, 7, true);
        leftEncoder = new Encoder(8, 9, false);
        
        // using meter is easier in ramsete command
        rightEncoder.setDistancePerPulse(Units.feetToMeters(distancePerPulse));
        leftEncoder.setDistancePerPulse(Units.feetToMeters(distancePerPulse));

        SmartDashboard.putData("Drivebase Left Encoder", leftEncoder);
        SmartDashboard.putData("Drivebase Right Encoder", rightEncoder);
    }

    public double getLeftDistance() {
        return leftEncoder.getDistance();
    }

    public double getRightDistance() {
        return rightEncoder.getDistance();
    }

    public double getRightVelocity() {
        return rightEncoder.getRate() ;
    }

    public double getLeftVelocity() {
        return leftEncoder.getRate();
    }
    public void zeroDistance() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getDistancePerPulse() {
        return distancePerPulse;
    }
}
