// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrainTestSubsystem extends SubsystemBase {
  WPI_TalonFX leftMaster = new WPI_TalonFX(DriveConstants.kLeftMotorMasterPort);
  WPI_TalonFX rightMaster = new WPI_TalonFX(DriveConstants.kRightMotorMasterPort);
  WPI_TalonFX leftSlave = new WPI_TalonFX(DriveConstants.kLeftMotorSlavePort);
  WPI_TalonFX rightSlave = new WPI_TalonFX(DriveConstants.kRightMotorSlavePort);

  public AHRS gyro = new AHRS(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.75997, 0.074198, 0.013752);

  // PIDController leftPidController = new PIDController(0.10775, 0, 0);
  // PIDController rightPidController = new PIDController(0.10775, 0, 0);
  PIDController rightPidController = new PIDController(8.5, 0, 0);
  PIDController leftPidController = new PIDController(8.5, 0, 0);

  Pose2d pose;

  private final DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  /** Creates a new DriveTrainTestSubsystem. */
  public DriveTrainTestSubsystem() {
    drive.setSafetyEnabled(false);
    leftSlave.set(ControlMode.Follower, leftMaster.getDeviceID());
    rightSlave.set(ControlMode.Follower, rightMaster.getDeviceID());

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);
    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);

    gyro.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftMaster.getSelectedSensorVelocity()  / 6 * Math.PI,
        rightMaster.getSelectedSensorVelocity() / 6 * Math.PI);
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPidController;
  }

  public PIDController getRightPIDController() {
    return rightPidController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getpose() {
    return pose;
  }

  public void setOuput(double leftVolts, double rightVolts) {
    leftMaster.set(leftVolts);
    rightMaster.set(rightVolts);
    drive.feed();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // System.out.println(gyro.getAngle());
    pose = odometry.update(getHeading(), leftMaster.getSelectedSensorVelocity()  / 6 * Math.PI,
        rightMaster.getSelectedSensorVelocity()  / 6 * Math.PI);
  }

  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void setNeutralMode(NeutralMode mode) {
    leftMaster.setNeutralMode(mode);
    leftSlave.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    rightSlave.setNeutralMode(mode);
  }

  public void setMotorSafety(boolean turnOn) {
    drive.setSafetyEnabled(turnOn);
  }

  public boolean isSafetyEnabled() {
    return drive.isSafetyEnabled();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void resetEncoders() {
    rightMaster.setSelectedSensorPosition(0);
    leftMaster.setSelectedSensorPosition(0);
  }
}
