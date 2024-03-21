// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Utilities.PID;
import frc.robot.Utilities.PolarVector;

public class Swerve extends Command {

  private final Drive drive;
  private final Supplier<PolarVector> desired_velocity_function;
  private final Supplier<Double> angular_velocity_function;
  private final Supplier<Boolean> maximize_velocity_function; 
  //private final Supplier<Boolean> zero_gyro_function;
  private final Supplier<Boolean> field_orient_function;
  private Rotation2d robot_rotation = new Rotation2d(0);

  private Rotation2d target_rotation = Rotation2d.fromDegrees(0.0);
  private Rotation2d antidrift_angle = Rotation2d.fromDegrees(0.0);

  private final PID robot_rotation_pid = new PID(
    0.2,
    0.0,
    0.0
  );

  /** Creates a new Swerve. */
    public Swerve(
        Drive drive,
        Supplier<PolarVector> desired_velocity_function,
        Supplier<Double> angular_veloctity_function,
        Supplier<Boolean> maximize_velocity_function,
        //Supplier<Boolean> zero_gyro_function,
        Supplier<Boolean> field_orient_function
      ) {
      this.drive = drive;
      this.desired_velocity_function = desired_velocity_function;
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

      if(field_orient_function.get()){
        drive.attempt_set_team();
        drive.attempt_field_orient();
      }

      double desired_angular_velocity = angular_velocity_function.get();

      // target_rotation = target_rotation.plus(Rotation2d.fromDegrees(desired_angular_velocity));

      // desired_angular_velocity = MathUtil.clamp(
      //   robot_rotation_pid.out(Pid,0.0,0.0),
      //   -1.0,
      //   1.0
      // );

      boolean maximize_velocity = maximize_velocity_function.get();

      PolarVector desired_velocity = desired_velocity_function.get();

      if(desired_velocity.length < 0.05){
        //maximize_velocity = false;
      }

      robot_rotation = drive.get_driver_angle();

      // if(Math.abs(desired_angular_velocity) > 0 || (antidrift_angle.getDegrees() == 0)){
      //   //drive.get_rotation_pid(robot_rotation);
      //   antidrift_angle = robot_rotation;
      // } else {
      //   desired_angular_velocity = drive.get_rotation_pid_antidrift(antidrift_angle); 
      // }

      PolarVector field_oriented_velocity = new PolarVector(desired_velocity.angle.minus(robot_rotation), desired_velocity.length);

      // ChassisSpeeds speeds = new ChassisSpeeds(
      //   field_oriented_velocity.x() * Constants.DriveConstants.max_speed_meters_per_second,
      //   field_oriented_velocity.y() * Constants.DriveConstants.max_speed_meters_per_second,
      //   desired_angular_velocity * Constants.DriveConstants.max_radians_per_second);

      // drive.drive_from_chassis_speeds(speeds);

      PolarVector[] wheel_velocities = drive.calculate_wheel_velocities(field_oriented_velocity, MathUtil.clamp(desired_angular_velocity,-1.0,1.0));

      double multiplier = Drive.calculate_multiplier(wheel_velocities);
      
      drive.run_wheels(wheel_velocities, multiplier);

      SmartDashboard.putNumber("desired_speed", desired_velocity.length);

      SmartDashboard.putNumberArray("speeds", new double[] {wheel_velocities.length} );

      SmartDashboard.putNumber("multiplier", multiplier);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      drive.stop_wheels();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
