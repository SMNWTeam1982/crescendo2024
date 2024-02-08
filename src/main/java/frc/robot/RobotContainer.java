// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Swerve.Drive;

import frc.robot.Commands.*;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }
  private Drive swerve_drive = new Drive();
  private final CommandXboxController drive_controller = new CommandXboxController(0);

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    
    return new SequentialCommandGroup(
      new FieldOrient(swerve_drive)
    );

  }
}
