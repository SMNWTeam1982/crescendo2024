// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Utilities.PolarVector;
import frc.robot.Utilities.Tracking;
import frc.robot.Utilities.Vector;

public class Swerve extends Command {

  private final Drive drive;
  private final Supplier<Vector> joystick_input_function;
  private final Supplier<Double> angular_velocity_function;
  private final Supplier<Boolean> maximize_velocity_function; 
  //private final Supplier<Boolean> zero_gyro_function;
  private final Supplier<Boolean> field_orient_function;
  private boolean started_on_blue_side = false;

  /** Creates a new Swerve. */
    public Swerve(
        Drive drive,
        Supplier<Vector> joystick_input_function,
        Supplier<Double> angular_veloctity_function,
        Supplier<Boolean> maximize_velocity_function,
        //Supplier<Boolean> zero_gyro_function,
        Supplier<Boolean> field_orient_function
      ) {
      this.drive = drive;
      this.joystick_input_function = joystick_input_function;
      this.angular_velocity_function = angular_veloctity_function;
      this.maximize_velocity_function = maximize_velocity_function;
      //this.zero_gyro_function = zero_gyro_function;
      this.field_orient_function = field_orient_function;
      addRequirements(drive);
      // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // if(zero_gyro_function.get()){
      //   drive.zero_gyro();
      // }

      Vector inputs = joystick_input_function.get();

      double y = inputs.y;
      if(started_on_blue_side){
        y = -y;
      }
      double x = inputs.x;
      if(started_on_blue_side){
        x = -x;
      }
      double speed = Math.sqrt(x * x + y * y);
      if(speed > 1.0){speed = 1.0;}
      if(speed < 0.1){speed = 0.0;}

      PolarVector desired_velocity = new PolarVector(
        Rotation2d.fromRadians(Math.atan2(y,x)).plus(Rotation2d.fromDegrees(90.0)),
        speed
      );

      if(field_orient_function.get()){
        started_on_blue_side = Limelight.get_field_position().getX() < 0.0;
        drive.attempt_field_orient();
      }

      double desired_angular_velocity = angular_velocity_function.get();

      boolean maximize_velocity = maximize_velocity_function.get();

      if(desired_velocity.length < 0.01){
        maximize_velocity = false;
      }
      Rotation2d robot_rotation = drive.get_gyro_angle();

      SmartDashboard.putNumber("robot heading", drive.get_gyro_angle().getDegrees());

      PolarVector field_oriented_velocity = new PolarVector(desired_velocity.angle.minus(robot_rotation), desired_velocity.length);

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
