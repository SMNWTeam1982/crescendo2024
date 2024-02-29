// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Utilities.PolarVector;
import frc.robot.Utilities.Vector;
import frc.robot.Commands.*;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }
  private final Drive swerve_drive = new Drive();
  private final Shooter shooter = new Shooter(13, 11, 12);
  private final Intake intake = new Intake(9, 10);
  private final XboxController drive_controller = new XboxController(0);
  private final XboxController shooter_controller = new XboxController(1);

  private void configureBindings() {
    swerve_drive.setDefaultCommand(
      new Swerve(
        swerve_drive,
        () -> {
          Vector inputs = new Vector(drive_controller.getLeftX(), -drive_controller.getLeftY());

          double speed = Math.sqrt(inputs.x * inputs.x + inputs.y * inputs.y);
          if(speed > 1.0){speed = 1.0;}
          if(speed < 0.1){speed = 0.0;}
    
          PolarVector desired_velocity = new PolarVector(
            Rotation2d.fromRadians(Math.atan2(inputs.y,inputs.x)).plus(Rotation2d.fromDegrees(90.0)),
            speed
          );

          return desired_velocity;
        },
        () -> {
          double x = drive_controller.getRightX();
          if(Math.abs(x) < 0.1){x = 0.0;}
          return x;
        },
        () -> {return (drive_controller.getRightTriggerAxis() < 0.1);},
        // () -> {return drive_controller.getAButton();},
        () -> {return drive_controller.getYButtonReleased();}
      )
    );
    shooter.setDefaultCommand(
      shooter.shooter_command(
        () -> {return (shooter_controller.getRightTriggerAxis() > 0.1);},
        () -> {
          return Constants.ShooterConstants.shooter_load_angle.plus(Limelight.get_speaker_angle()); // math not implemented yet
        }
      )
    );

    intake.setDefaultCommand(
      intake.get_intake_command(
        () -> {
          return shooter_controller.getLeftY();
        },
        () -> {
          if(shooter_controller.getYButton()){
            return Constants.IntakeConstants.amp_scoring_angle;
          }
          if(shooter_controller.getBButton()){
            return Constants.IntakeConstants.deployed_angle;
          }
          if(shooter_controller.getXButton()){
            return Constants.IntakeConstants.handoff_angle;
          }
          return null;
        },
        () -> {
          return shooter_controller.getAButton();
        }
      )
    );
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new PrintCommand("starting auto"),
      new Calibrate(swerve_drive,intake,shooter)
    );
  }
}
