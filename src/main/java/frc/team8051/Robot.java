package frc.team8051;

import frc.team8051.subsystems.DifferentialDrivebase;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private DifferentialDrivebase drivebase = new DifferentialDrivebase();
  private RamseteCommand rameseteDrive = 
  new RamseteCommand(
    trajectory, drivebase::getPose,  new RamseteController(2, .7), 
    drivebase.getFeedforward(), drivebase.getKinematics(), drivebase::getSpeeds, 
    drivebase.getLeftPIDController(), drivebase.getRightPIDController(), 
    drivebase::setOutputVolts, drivebase
  );

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
  public void teleopPeriodic() {

  }


  @Override
  public void testPeriodic() {

  }
}
