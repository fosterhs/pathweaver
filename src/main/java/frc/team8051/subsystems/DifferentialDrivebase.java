package frc.team8051.subsystems;

import frc.team8051.sensors.DrivebaseEncoder;
import frc.team8051.sensors.Gyro;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

final class Cermaics {
  final class FeedForwardConstants {
    // 0.808, 0.952, 0.144
    public static final double Ks = 0.808;
    public static final double Kv = 0.952;
    public static final double Ka = 0.144;
  }
  final class PIDConstants {
    // 6.43, 0, 0
    public static final double Kp = 6.43;
    public static final double Ki = 0.00;
    public static final double Kd = 0.00;
  }
}

final class Carpet {
  final class FeedForwardConstants {
    // 0.992, 0.959, 0.205
    public static final double Ks = 0.992;
    public static final double Kv = 0.959;
    public static final double Ka = 0.205;
  }
  final class PIDConstants {
    // 8.99, 0, 0
    public static final double Kp = 8.99;
    public static final double Ki = 0.00;
    public static final double Kd = 0.00;
  }
}

public class DifferentialDrivebase extends SubsystemBase {

    private final WPI_VictorSPX leftMotor = new WPI_VictorSPX(15);
    private final WPI_VictorSPX rightMotor =  new WPI_VictorSPX(14);
    private final DifferentialDrive differentialDrive;
    private final DrivebaseEncoder encoders = new DrivebaseEncoder();;
    private final Gyro gyro = new Gyro();

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.685);

    private Pose2d pose = new Pose2d(0.0, 0.0, getHeading());

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), pose);
  
    private final SimpleMotorFeedforward feedforward = 
    new SimpleMotorFeedforward(
      Carpet.FeedForwardConstants.Ks,
      Carpet.FeedForwardConstants.Kv,
      Carpet.FeedForwardConstants.Ka
    );

    private final PIDController leftPIDController = new PIDController(Carpet.PIDConstants.Kp, 0, 0);
    private final PIDController rightPIDController = new PIDController(Carpet.PIDConstants.Kp, 0, 0);
    
    public DifferentialDrivebase() {
        differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

        zeroDistance();

        SmartDashboard.putData("Left PID Controller", leftPIDController);
        SmartDashboard.putData("Right PID Controller", rightPIDController);
    }

    public DifferentialDrive getDifferentialDrive() {
        return differentialDrive;
    }

    public void zeroHeading() {
      gyro.reset();
    }

    public void zeroDistance() {
      encoders.zeroDistance();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getHeading(), 360));
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
          encoders.getLeftVelocity(), 
          encoders.getRightVelocity()
        );
      }
    
      public DifferentialDriveKinematics getKinematics() {
        return kinematics;
      }
    
      public Pose2d getPose() {
        return pose;
      }
    
      public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
      }
    
      public PIDController getLeftPIDController() {
        return leftPIDController;
      }
    
      public PIDController getRightPIDController() {
        return rightPIDController;
      }
    
      public void setVolts(double leftVolts, double rightVolts) {
        leftMotor.setVoltage(leftVolts);
        rightMotor.setVoltage(-rightVolts);
        differentialDrive.feed();
      }
    
      public void reset() {
        zeroDistance();
        zeroHeading();
        odometry.resetPosition(new Pose2d(), getHeading());
      }
    
      @Override
      public void periodic() {
        pose = odometry.update(
          getHeading(), 
          encoders.getLeftDistance(), 
          encoders.getRightDistance()
        );
        SmartDashboard.putNumber("Gyro_Heading", gyro.getHeading());
      }
    
}
