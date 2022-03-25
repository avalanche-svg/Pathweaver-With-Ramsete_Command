// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveTrainTestSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.PathMatcher;
import java.nio.file.Paths;
import java.util.Arrays;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    public final DriveTrainTestSubsystem drive = new DriveTrainTestSubsystem();
    private Robot m_robot;;

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        drive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () -> drive.arcadeDrive(
                                -m_driverController.getLeftY(), m_driverController.getLeftX()),
                        drive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Drive at half speed when the right bumper is held
        new JoystickButton(m_driverController, Button.kRightBumper.value)
                .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
                .whenReleased(() -> m_robotDrive.setMaxOutput(1));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        TrajectoryConfig config = new TrajectoryConfig(Units.inchesToMeters(10000.0), Units.inchesToMeters(6000.0));
        config.setKinematics(drive.getKinematics());
        /*
          Trajectory trajectory =
          TrajectoryGenerator.generateTrajectory(Arrays.asList(new Pose2d(), new
          Pose2d(6, 6, new Rotation2d())), config
          );
         */
       
         String trajectory1 = "output/testpath.wpilib.json"; // first thing to change is add output/ to make it "Paths/ouput/PathWeaverTest.wpilib.json"
        Trajectory PathWeaverTrajectory1 = new Trajectory();

        try {
            Path PathweaverTestPath = Filesystem.getDeployDirectory().toPath().resolve(trajectory1);
            PathWeaverTrajectory1 = TrajectoryUtil.fromPathweaverJson(PathweaverTestPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectory1, ex.getStackTrace());
        }
        RamseteCommand command = new RamseteCommand(PathWeaverTrajectory1, drive::getpose,
                new RamseteController(2, 0.7), drive.getFeedForward(), drive.getKinematics(), drive::getSpeeds,
                drive.getLeftPIDController(), drive.getRightPIDController(), drive::setOuput, drive);
        return command;
        
        // Create a voltage constraint to ensure we don't accelerate too fast
        /*
         * var autoVoltageConstraint =
         * new DifferentialDriveVoltageConstraint(
         * new SimpleMotorFeedforward(
         * DriveConstants.ksVolts,
         * DriveConstants.kvVoltSecondsPerMeter,
         * DriveConstants.kaVoltSecondsSquaredPerMeter),
         * DriveConstants.kDriveKinematics,
         * 10);
         * 
         * // Create config for trajectory
         * TrajectoryConfig config =
         * new TrajectoryConfig(
         * AutoConstants.kMaxSpeedMetersPerSecond,
         * AutoConstants.kMaxAccelerationMetersPerSecondSquared)
         * // Add kinematics to ensure max speed is actually obeyed
         * .setKinematics(DriveConstants.kDriveKinematics)
         * // Apply the voltage constraint
         * .addConstraint(autoVoltageConstraint);
         * 
         * // An example trajectory to follow. All units in meters.
         * Trajectory exampleTrajectory =
         * TrajectoryGenerator.generateTrajectory(
         * // Start at the origin facing the +X direction
         * new Pose2d(0, 0, new Rotation2d(0)),
         * // Pass through these two interior waypoints, making an 's' curve path
         * List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
         * // End 3 meters straight ahead of where we started, facing forward
         * new Pose2d(3, 0, new Rotation2d(0)),
         * // Pass config
         * config);
         * 
         * RamseteCommand ramseteCommand =
         * new RamseteCommand(
         * exampleTrajectory,
         * m_robotDrive::getPose,
         * new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
         * new SimpleMotorFeedforward(
         * DriveConstants.ksVolts,
         * DriveConstants.kvVoltSecondsPerMeter,
         * DriveConstants.kaVoltSecondsSquaredPerMeter),
         * DriveConstants.kDriveKinematics,
         * m_robotDrive::getWheelSpeeds,
         * new PIDController(DriveConstants.kPDriveVel, 0, 0),
         * new PIDController(DriveConstants.kPDriveVel, 0, 0),
         * // RamseteCommand passes volts to the callback
         * m_robotDrive::tankDriveVolts,
         * m_robotDrive);
         * 
         * // Reset odometry to the starting pose of the trajectory.
         * m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
         * 
         * // Run path following command, then stop at the end.
         * return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
         */
    }
    public DriveTrainTestSubsystem getDriveTrainTestSubsystem() {
        return drive;
      }
}
