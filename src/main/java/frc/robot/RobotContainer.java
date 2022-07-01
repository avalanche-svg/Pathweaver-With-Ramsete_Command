// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.AutonomousPathweaverTest;
import frc.robot.Commands.Drivestop;
import frc.robot.Commands.TrajectoryGeneratorTestCommand;
import frc.robot.Constants.OIConstants;
// import frc.robot.subsystems.DriveSubsystem;
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
import java.util.List;

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
    // private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    public final DriveTrainTestSubsystem drive = new DriveTrainTestSubsystem();
    private Robot m_robot;;

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    private final AutonomousPathweaverTest autonomousPathweaverTest = new AutonomousPathweaverTest(drive);
    private final TrajectoryGeneratorTestCommand trajectoryGenearatorTestCommand = new TrajectoryGeneratorTestCommand(drive);
    private final Drivestop stopDrive = new Drivestop(drive);
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
        new JoystickButton(m_driverController, Button.kA.value).whenPressed(autonomousPathweaverTest);
        new JoystickButton(m_driverController, Button.kB.value).whenPressed(trajectoryGenearatorTestCommand);
        // Drive at half speed when the right bumper is held
        // new JoystickButton(m_driverController, Button.kRightBumper.value)
        //         .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
        //         .whenReleased(() -> m_robotDrive.setMaxOutput(1));
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
            PathWeaverTrajectory1 =
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
        RamseteCommand command = new RamseteCommand(PathWeaverTrajectory1, drive::getpose,
                new RamseteController(2, 0.7), drive.getFeedForward(), drive.getKinematics(), drive::getSpeeds,
                drive.getLeftPIDController(), drive.getRightPIDController(), drive::setOuput, drive);
                // System.out.println("motor safety:" + drive.isSafetyEnabled());
                
        return command;
        
    }
    public void updateTelemetry() {
        SmartDashboard.putNumber("Robot Pose X", drive.getpose().getX());
        SmartDashboard.putNumber("Robot Pose Y", drive.getpose().getY());
      }
    public DriveTrainTestSubsystem getDriveTrainTestSubsystem() {
        return drive;
      }
    public Drivestop getdrivestop() {
        return stopDrive;
    }
}
