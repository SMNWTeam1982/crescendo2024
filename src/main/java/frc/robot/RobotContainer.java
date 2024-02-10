// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Utilities.PolarVector;
import frc.robot.Commands.*;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }
  private Drive swerve_drive = new Drive();
  private Shooter shooter = new Shooter(-1, -1, -1);
  private final Intake intake = new Intake(-1, -1);
  private final XboxController drive_controller = new XboxController(0);
  private final XboxController shooter_controller = new XboxController(1);

  private void configureBindings() {
    swerve_drive.setDefaultCommand(
      new Swerve(
        swerve_drive,
        () -> {
          double x = drive_controller.getLeftX();
          double y = drive_controller.getLeftY();
          double trigger = drive_controller.getRightTriggerAxis();
          double speed = Math.sqrt(x * x + y * y);
          if(speed > 1.0){speed = 1.0;}
          if(speed < 0.1){speed = 0.0;}
          if(trigger > 0.1){speed /= 3.0;}
          PolarVector linear_velocity = new PolarVector(
            Rotation2d.fromRadians(Math.atan2(y,x)),
            speed
          );
          return linear_velocity;
        },
        () -> {
          double x = drive_controller.getRightX();
          if(x < 0.2){x = 0.0;}
          return x;
        },
        () -> {
          return (drive_controller.getRightTriggerAxis() > 0.1);
        }
      )
    );
    shooter.setDefaultCommand(
      shooter.shooter_command(
        () -> {
          return shooter_controller.getAButton();
        },
        () -> {
          return Constants.ShooterConstants.shooter_start_angle; // math not implemented yet
        }
      )
    );

    intake.setDefaultCommand(
      intake.get_intake_command(
        () -> {
          return shooter_controller.getLeftY();
        },
        () -> {
          if(shooter_controller.getBButton()){
            return Constants.IntakeConstants.deployed_angle;
          }else{
            return Constants.IntakeConstants.handoff_angle;
          }
        }
      )
    );
  }

  public Command getAutonomousCommand() {
    
    return new SequentialCommandGroup(
      new FieldOrient(swerve_drive)
    );

  }
}
