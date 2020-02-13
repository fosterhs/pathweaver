package frc.team8051.subsystems;

import frc.team8051.sensors.DrivebaseEncoder;
import frc.team8051.sensors.Gyro;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
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

// Wrapper Class for differential drivebase
public class DifferentialDrivebase extends Subsystem {
    private final WPI_VictorSPX leftMotor = new WPI_VictorSPX(15);
    private final WPI_VictorSPX rightMotor =  new WPI_VictorSPX(14);
    private final DifferentialDrive differentialDrive;
    private final DrivebaseEncoder encoders = new DrivebaseEncoder();;
    private final Gyro gyro = new Gyro();

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(31));

    private Pose2d pose = new Pose2d(0, 0, getHeading());

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), pose);
  
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);
    private final PIDController leftPIDController = new PIDController(0, 0, 0);
    private final PIDController rightPIDController = new PIDController(0, 0, 0);
    
    public DifferentialDrivebase() {
        differentialDrive = new DifferentialDrive(leftMotor, rightMotor);
        gyro.calibrate();
        gyro.reset();
        encoders.zeroEncoder();
        SmartDashboard.putData("Left PID Controller", leftPIDController);
        SmartDashboard.putData("Right PID Controller", rightPIDController);
    }

    @Override
    protected void initDefaultCommand() {

    }

    public DifferentialDrive getDifferentialDrive() {
        return differentialDrive;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro.getHeading());
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(encoders.getLeftVelocity(), 
            encoders.getRightVelocity());
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
    
      public void setOutputVolts(double leftVolts, double rightVolts) {
        differentialDrive.tankDrive(leftVolts/12, rightVolts/12);
        // leftMotor.set(leftVolts / 12);
        // rightMotor.set(rightVolts / 12);
      }
    
      public void reset() {
        
        odometry.resetPosition(new Pose2d(), getHeading());
      }
    
      @Override
      public void periodic() {
        pose = odometry.update(getHeading(), 
            Units.feetToMeters(encoders.getLeftDistance()), 
                Units.feetToMeters(encoders.getRightDistance()));
      }
    
}
