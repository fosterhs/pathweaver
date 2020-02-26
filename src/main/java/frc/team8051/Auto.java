package frc.team8051;

import java.util.List;

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
import frc.team8051.subsystems.DifferentialDrivebase;

public class Auto {
    DifferentialDrivebase drivebase;
    public Auto(DifferentialDrivebase drivebase) {
        this.drivebase = drivebase;
    }

    public SequentialCommandGroup getSimpleCommand(
        double configVelocity, 
        double configAcceleration,
        List<Translation2d> waypoints, 
        Pose2d startPose, Pose2d endPose){
        
        drivebase.reset();
        DifferentialDriveVoltageConstraint autoVoltageConstraint = 
        new DifferentialDriveVoltageConstraint(
            drivebase.getFeedforward(), 
            drivebase.getKinematics(), 
            12
        );
      
        TrajectoryConfig trajectoryConfig = 
        new TrajectoryConfig(
            configVelocity,  
            configAcceleration
        )
        .setKinematics(drivebase.getKinematics())
        .addConstraint(autoVoltageConstraint);
      
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          startPose,
          waypoints,
          endPose,
          trajectoryConfig
        );
        
        RamseteCommand ramsete =
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
      
        return ramsete.andThen(()->drivebase.setVolts(0, 0));
    }

    public SequentialCommandGroup getSimpleCommand() {
        return getSimpleCommand(
            1.0, 
            1.0,  
            List.of(new Translation2d(.5, 0)),
            new Pose2d(0, 0, new Rotation2d()),
            new Pose2d(1, 0, new Rotation2d())
        );
    }
}