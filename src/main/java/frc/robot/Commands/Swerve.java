// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Utilities.PolarVector;

public class Swerve extends Command {

  private final Drive drive;
  private final Supplier<PolarVector> linear_velocity_function;
  private final Supplier<Double> angular_velocity_function;

  /** Creates a new Swerve. */
    public Swerve(Drive drive, Supplier<PolarVector> linear_velocty_function, Supplier<Double> angular_veloctity_function) {
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
      PolarVector desired_velocity = linear_velocity_function.get();
      double desired_angular_velocity = angular_velocity_function.get();
      Rotation2d gyro_angle = drive.get_gyro_angle();

      PolarVector field_oriented_velocity = new PolarVector(desired_velocity.angle.plus(gyro_angle), desired_velocity.length);

      // front_right_wheel.swerve(field_oriented_velocity.angle, field_oriented_velocity.length, angular_velocity);;
      // front_left_wheel.swerve(field_oriented_velocity.angle, field_oriented_velocity.length, angular_velocity);;
      // back_left_wheel.swerve(field_oriented_velocity.angle, field_oriented_velocity.length, angular_velocity);;
      // back_right_wheel.swerve(field_oriented_velocity.angle, field_oriented_velocity.length, angular_velocity);;

      

      if(!maximize_speed){
          front_right_wheel.run(front_right_desired_velocty, 1.0);
          front_left_wheel.run(front_left_desired_velocty, 1.0);
          back_left_wheel.run(back_left_desired_velocty, 1.0);
          back_right_wheel.run(back_right_desired_velocty, 1.0);
          return;
      }

      double highest = 0.0;
      if(front_right_desired_velocty.length > highest){
          highest = front_right_desired_velocty.length;
      }

      if(front_left_desired_velocty.length > highest){
          highest = front_left_desired_velocty.length;
      }

      if(back_left_desired_velocty.length > highest){
          highest = back_left_desired_velocty.length;
      }

      if(back_right_desired_velocty.length > highest){
          highest = back_right_desired_velocty.length;
      }
      
      double multiplier = 1.0 / highest;

      if(highest < 0.01){
          multiplier = 1.0;
      }
      
      front_right_wheel.run(front_right_desired_velocty, multiplier);
      front_left_wheel.run(front_left_desired_velocty, multiplier);
      back_left_wheel.run(back_left_desired_velocty, multiplier);
      back_right_wheel.run(back_right_desired_velocty, multiplier);
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
