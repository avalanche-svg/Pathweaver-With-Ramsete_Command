// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrainTestSubsystem;

public class TrajectoryGeneratorTestCommand extends CommandBase {
  DriveTrainTestSubsystem drive;

  Trajectory Trajectory = new Trajectory();

  /** Creates a new TrajectoryGeneratorTestCommand. */
  public TrajectoryGeneratorTestCommand(DriveTrainTestSubsystem drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TrajectoryConfig config = new TrajectoryConfig(Units.inchesToMeters(10000.0), Units.inchesToMeters(6000.0));
    config.setKinematics(drive.getKinematics());
   
    Trajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 3, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                List.of(new Translation2d(1,3), new Translation2d(2,3)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 3, new Rotation2d(0)),
                // Pass config
                config);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RamseteCommand command = new RamseteCommand(Trajectory, drive::getpose,
                new RamseteController(2, 0.7), drive.getFeedForward(), drive.getKinematics(), drive::getSpeeds,
                drive.getLeftPIDController(), drive.getRightPIDController(), drive::setOuput, drive);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
