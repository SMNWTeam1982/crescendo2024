// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Utilities.PolarVector;

public class Swerve extends Command {

  private final Drive drive;
  private final Supplier<PolarVector> linear_velocity_function;
  private final Supplier<Double> angular_velocity_function;
  private final Supplier<Boolean> maximize_velocity_function; 
  private final Supplier<Boolean> zero_gyro_function;

  /** Creates a new Swerve. */
    public Swerve(
        Drive drive,
        Supplier<PolarVector> linear_velocty_function,
        Supplier<Double> angular_veloctity_function,
        Supplier<Boolean> maximize_velocity_function,
        Supplier<Boolean> zero_gyro_function
      ) {
      this.drive = drive;
      this.linear_velocity_function = linear_velocty_function;
      this.angular_velocity_function = angular_veloctity_function;
      this.maximize_velocity_function = maximize_velocity_function;
      this.zero_gyro_function = zero_gyro_function;
      addRequirements(drive);
      // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

      if(zero_gyro_function.get()){
        drive.zero_gyro();
      }

      PolarVector desired_velocity = linear_velocity_function.get();
      double desired_angular_velocity = angular_velocity_function.get();
      boolean maximize_velocity = maximize_velocity_function.get();
      if(desired_velocity.length < 0.01){
        maximize_velocity = false;
      }
      Rotation2d gyro_angle = drive.get_gyro_angle();

      SmartDashboard.putNumber("robot heading", drive.get_gyro_angle().getDegrees());

      PolarVector field_oriented_velocity = new PolarVector(desired_velocity.angle.minus(gyro_angle), desired_velocity.length);

      PolarVector[] wheel_velocities = drive.calculate_wheel_velocities(field_oriented_velocity, desired_angular_velocity);

      if(!maximize_velocity){
          drive.run_wheels(wheel_velocities, 1.0);
          return;
      }

      double highest = 0.0;
      for(PolarVector wheel_velocity : wheel_velocities){
        if( wheel_velocity.length > highest ){
          highest = wheel_velocity.length;
        }
      }

      double multiplier = 1.0 / highest;

      if(highest < 0.01){
          multiplier = 1.0;
      }
      
      drive.run_wheels(wheel_velocities, multiplier);

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
