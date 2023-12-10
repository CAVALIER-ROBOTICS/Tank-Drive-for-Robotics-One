// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  XboxController controller = new XboxController(0);
  DriveSubsystem driveSub =  new DriveSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSub.setDefaultCommand(new DriveCommand(driveSub, controller::getLeftY, controller::getRightY));
    
    // driveSub.setDefaultCommand(new DriveCommand(
    //   driveSub,
    //   () -> -controller.getRawAxis(1),
    //   () -> controller.getRawAxis(0)
    // ));

    configureBindings();
  }

  private void configureBindings() {

  }
}
