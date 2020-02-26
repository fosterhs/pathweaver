package frc.team8051;

/*

import frc.team8051.subsystems.DifferentialDrivebase;

import java.util.List;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {  
  private DifferentialDrivebase drivebase = new DifferentialDrivebase();
  private Auto auto = new Auto(drivebase);
  private SequentialCommandGroup autoCommand = auto.getSimpleCommand();
  
  private Translation2d wayPoint = new Translation2d(0.5, 0);
  private double V = 1.0;
  private double A = 1.0;
  private Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
  private Pose2d endPose = new Pose2d(1, 0, new Rotation2d(0));
  
  public Robot() {
    SmartDashboard.putBoolean("New_Trajectory", false);

    SmartDashboard.putNumber("V", 1.0);
    SmartDashboard.putNumber("A", 1.0);
    
    SmartDashboard.putNumber("WX", 0.5);
    SmartDashboard.putNumber("WY", 0.0);

    SmartDashboard.putNumber("InitialX", 0.0);
    SmartDashboard.putNumber("InitialRot", 0.0);
    SmartDashboard.putNumber("InitialY", 0.0);

    SmartDashboard.putNumber("FinalX", 1.0);
    SmartDashboard.putNumber("FinalRot", 0);
    SmartDashboard.putNumber("FinalY", 0.0);

  }

  @Override
  public void robotInit() {
    // List<Trajectory.State> li = trajectory.getStates();
    // System.out.println("Total states: " + li.size());
    // for(Trajectory.State v : li) {
    //   System.out.println(v);
    // }
  }

  @Override
  public void robotPeriodic() {
    if(SmartDashboard.getBoolean("New_Trajectory", false)) {
      if(autoCommand.isScheduled()) CommandScheduler.getInstance().cancelAll();
      V = SmartDashboard.getNumber("V", 1.0);
      A = SmartDashboard.getNumber("A", 1.0);
      
      wayPoint = new Translation2d( 
        SmartDashboard.getNumber("WX", 0.5),
        SmartDashboard.getNumber("WY", 0.0)
      );
  
      startPose = new Pose2d(
        SmartDashboard.getNumber("InitialX", 0.0),
        SmartDashboard.getNumber("InitialY", 0.0),
        Rotation2d.fromDegrees(SmartDashboard.getNumber("InitialRot", 0.0))
      );

      endPose = new Pose2d(
        SmartDashboard.getNumber("FinalX", 1.0),
        SmartDashboard.getNumber("FinalY", 0.0),
        Rotation2d.fromDegrees(SmartDashboard.getNumber("FinalRot", 0))
      );

      SmartDashboard.putBoolean("New_Trajectory", false);

      autoCommand = auto.getSimpleCommand(V, A, List.of(wayPoint), startPose, endPose);
    }

    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override 
  public void teleopInit() {
    autoCommand.schedule();
  }

  @Override
  public void teleopPeriodic() {

  }


  @Override
  public void testPeriodic() {

  }

  @Override
  public void disabledInit() {
    drivebase.reset();
    CommandScheduler.getInstance().cancelAll();
  }
}
*/

import java.util.List;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.team8051.subsystems.DifferentialDrivebase;

public class Robot extends TimedRobot {
  private DifferentialDrivebase drivebase = new DifferentialDrivebase();
  DifferentialDriveVoltageConstraint autoVoltageConstraint = 
  new DifferentialDriveVoltageConstraint(
   drivebase.getFeedforward(), 
   drivebase.getKinematics(), 
   10
  );

  // 2, 4 is good
  private TrajectoryConfig trajectoryConfig = new TrajectoryConfig(3, 3)
  .setKinematics(drivebase.getKinematics())
  .addConstraint(autoVoltageConstraint);

  private Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d()),
    List.of(
      new Translation2d(1, .25),
      new Translation2d(2, -.25)
    ),
    new Pose2d(3, 1, new Rotation2d()),
    trajectoryConfig
  );
  
  private RamseteCommand ramsete =
  new RamseteCommand(
    trajectory,
    drivebase::getPose, 
    new RamseteController(2.0, 0.7), 
    drivebase.getFeedforward(), 
    drivebase.getKinematics(), 
    drivebase::getWheelSpeeds, 
    drivebase.getLeftPIDController(), 
    drivebase.getRightPIDController(), 
    drivebase::setVolts, 
    drivebase
  );

  SequentialCommandGroup autoCommand = ramsete.andThen(()->drivebase.setVolts(0, 0));

  public Robot() {

  }

  @Override
  public void robotInit() {
    List<Trajectory.State> li = trajectory.getStates();
    System.out.println("Total states: " + li.size());
    for(Trajectory.State v : li) {
      System.out.println(v);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override 
  public void teleopInit() {
    
    if(autoCommand.isScheduled()) {
      CommandScheduler.getInstance().cancel(autoCommand);
    }
    drivebase.reset();
    autoCommand.schedule();
  }

  @Override
  public void teleopPeriodic() {
  }


  @Override
  public void testPeriodic() {

  }
}