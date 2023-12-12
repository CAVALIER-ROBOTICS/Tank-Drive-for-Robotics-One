// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  final double defaultDeadband = .02;

  // Make sure these IDs are correspond to the correct motors.
  // You can view motor IDs in Phoenix Tuner.
  
  TalonSRX left = new TalonSRX(1); //Front left
  TalonSRX right = new TalonSRX(2); //Front right
  TalonSRX leftFollower = new TalonSRX(3); //Back left
  TalonSRX rightFollower = new TalonSRX(4); //Back right

  public DriveSubsystem() {
    leftFollower.follow(left, FollowerType.PercentOutput);
    rightFollower.follow(right, FollowerType.PercentOutput);
    leftFollower.setInverted(InvertType.OpposeMaster);
    rightFollower.setInverted(InvertType.FollowMaster);
  }

  public DriveTrainState driveIK(double drivePercent, double steerPercent) {
    drivePercent = MathUtil.applyDeadband(drivePercent, defaultDeadband, 1);
    steerPercent = MathUtil.applyDeadband(steerPercent, defaultDeadband, 1);

    double leftDrivePercent = (drivePercent + steerPercent);
    double rightDrivePercent = (drivePercent - steerPercent);

    double absoluteDrive = Math.abs(drivePercent);
    double absoluteSteer = Math.abs(steerPercent);

    double greatestInput = Math.max(absoluteDrive, absoluteSteer);

    if (greatestInput == 0.0) {
      return DriveTrainState.zeroes();
    }

    double maxxed = (absoluteDrive + absoluteSteer) / greatestInput;

    leftDrivePercent /= maxxed;
    rightDrivePercent /= maxxed;

    return new DriveTrainState(leftDrivePercent, rightDrivePercent);
  }

  public void log(DriveTrainState state) {
    SmartDashboard.putNumber("Left Power Percent", state.getLeftPercent());
    SmartDashboard.putNumber("Right Power Percent", state.getRightPercent());
  }

  public void drive(double drive, double steer) {
    DriveTrainState state = driveIK(drive, steer);

    log(state);

    left.set(ControlMode.PercentOutput, state.getLeftPercent());
    right.set(ControlMode.PercentOutput, state.getRightPercent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
