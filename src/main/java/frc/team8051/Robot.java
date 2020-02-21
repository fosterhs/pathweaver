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
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private DifferentialDrivebase drivebase = new DifferentialDrivebase();
  DifferentialDriveVoltageConstraint autoVoltageConstraint = 
  new DifferentialDriveVoltageConstraint(
   drivebase.getFeedforward(), 
   drivebase.getKinematics(), 
   12
  );

  private TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2, 2)
  .setKinematics(drivebase.getKinematics())
  .addConstraint(autoVoltageConstraint);

  private Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
      new Translation2d(.5, 0)
      // new Translation2d(2, 0.5)
    ),
    new Pose2d(2, 0, Rotation2d.fromDegrees(90)),
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
