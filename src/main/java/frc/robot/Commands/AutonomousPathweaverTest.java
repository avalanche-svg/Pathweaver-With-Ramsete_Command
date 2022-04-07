// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrainTestSubsystem;

public class AutonomousPathweaverTest extends CommandBase {
  DriveTrainTestSubsystem drive;
  String trajectory1 = "output/testpath.wpilib.json"; // first thing to change is add output/ to make it "Paths/ouput/PathWeaverTest.wpilib.json"
  Trajectory PathWeaverTrajectory1 = new Trajectory();
  /** Creates a new AutonomousPathweaverTest. */
  public AutonomousPathweaverTest(DriveTrainTestSubsystem drive){
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetOdometry(PathWeaverTrajectory1.getInitialPose());
    try {
      Path PathweaverTestPath = Filesystem.getDeployDirectory().toPath().resolve(trajectory1);
      PathWeaverTrajectory1 = TrajectoryUtil.fromPathweaverJson(PathweaverTestPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectory1, ex.getStackTrace());
    }

    RamseteCommand command = new RamseteCommand(PathWeaverTrajectory1, drive::getpose,
    new RamseteController(2, 0.7), drive.getFeedForward(), drive.getKinematics(), drive::getSpeeds,
    drive.getLeftPIDController(), drive.getRightPIDController(), drive::setOuput, drive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("command ended");
    drive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
