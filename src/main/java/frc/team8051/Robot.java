package frc.team8051;

import frc.team8051.subsystems.DifferentialDrivebase;

import java.util.List;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private DifferentialDrivebase drivebase = new DifferentialDrivebase();
  private TrajectoryConfig trajectoryConfig = new TrajectoryConfig(0.50, 0.25);

  private Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d()),
    List.of(
      new Translation2d(0.5, -0.25)
    ),
    new Pose2d(1, 0, new Rotation2d()),
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

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override 
  public void teleopInit() {
    
    if(!autoCommand.isScheduled()) {
      drivebase.reset();
      autoCommand.schedule();
    }
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }


  @Override
  public void testPeriodic() {

  }
}
