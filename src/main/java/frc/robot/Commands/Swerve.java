// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.NewSwerve.SwerveSubsystem;
import frc.robot.Utilities.PolarVector;

public class Swerve extends Command {

  private final SwerveSubsystem drive;
  private final Supplier<PolarVector> linear_velocity_function;
  private final Supplier<Double> angular_velocity_function;

  /** Creates a new Swerve. */
  public Swerve(SwerveSubsystem drive, Supplier<PolarVector> linear_velocty_function, Supplier<Double> angular_veloctity_function) {
    this.drive = drive;
    this.linear_velocity_function = linear_velocty_function;
    this.angular_velocity_function = angular_veloctity_function;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PolarVector linear_velocity = linear_velocity_function.get();
    linear_velocity.length *= Constants.DriveConstants.wheel_max_speed_meters_per_second;
    double angular_velocity = angular_velocity_function.get() * Constants.DriveConstants.angular_velocity_multiplier;

    ChassisSpeeds chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      linear_velocity.x(),
      linear_velocity.y(),
      angular_velocity,
      drive.get_gyro_angle()
    );

    drive.set_chassis_speed(chassis_speeds);
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
