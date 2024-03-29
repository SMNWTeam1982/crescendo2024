// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.PID;

public class Shooter extends SubsystemBase {
  private final CANSparkMax pivot_motor;
  private final RelativeEncoder pivot_encoder;
  private final CANSparkMax upper_shoot_motor;
  private final CANSparkMax lower_shoot_motor;
  public Rotation2d target_angle = Constants.ShooterConstants.shooter_start_angle;
  private final PID pivot_pid = new PID(
    Constants.ShooterConstants.pivot_p,
    Constants.ShooterConstants.pivot_i,
    Constants.ShooterConstants.pivot_d
  );

  /** Creates a new Shooter. */
  public Shooter(int pivot_motor_channel, int upper_shoot_motor_channel, int lower_shoot_motor_channel) {
    upper_shoot_motor = new CANSparkMax(upper_shoot_motor_channel, MotorType.kBrushless);

    lower_shoot_motor = new CANSparkMax(lower_shoot_motor_channel, MotorType.kBrushless);

    pivot_motor = new CANSparkMax(pivot_motor_channel, MotorType.kBrushless);


    upper_shoot_motor.restoreFactoryDefaults();
    upper_shoot_motor.setOpenLoopRampRate(1.0);
    upper_shoot_motor.setClosedLoopRampRate(1.0);
    lower_shoot_motor.restoreFactoryDefaults();
    lower_shoot_motor.setOpenLoopRampRate(1.0);
    lower_shoot_motor.setClosedLoopRampRate(1.0);
    pivot_encoder = pivot_motor.getEncoder();
    pivot_encoder.setPosition(Constants.ShooterConstants.shooter_start_angle.getRotations());
  }

  public Command shooter_command(Supplier<Double> shoot_function, Supplier<Rotation2d> target_angle_function){
    return run(
      () -> {
        if(shoot_function.get()){
          set_shoot_motors(1.0);
        }else{
          set_shoot_motors(0.0);
        }

        set_pivot_motor(pivot_pid.out(get_shooter_angle().getDegrees(), target_angle_function.get().getDegrees(), 0.0));
        
      }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Rotation2d calculate_shooter_angle(Pose2d robot_pose){
    
  }

  public Rotation2d get_shooter_angle(){
    return Rotation2d.fromRotations(pivot_encoder.getPosition() * Constants.ShooterConstants.pivot_motor_rotations_to_shooter_rotations)
    .plus(Constants.ShooterConstants.shooter_start_angle);
  }

  public void set_shoot_motors(double speed){
    if(speed > 1.0){
      upper_shoot_motor.set(1.0 * Constants.ShooterConstants.upper_shoot_motor_multiplier);
      lower_shoot_motor.set(1.0 * Constants.ShooterConstants.lower_shoot_motor_multiplier);
      return;
    }
    if(speed < -1.0){
      upper_shoot_motor.set(1.0 * Constants.ShooterConstants.upper_shoot_motor_multiplier);
      lower_shoot_motor.set(1.0 * Constants.ShooterConstants.lower_shoot_motor_multiplier);
      return;
    }
    upper_shoot_motor.set(speed * Constants.ShooterConstants.upper_shoot_motor_multiplier);
    lower_shoot_motor.set(speed * Constants.ShooterConstants.lower_shoot_motor_multiplier);
  }

  public void set_pivot_motor(double speed){
    if(speed > 1.0){
      pivot_motor.set(1.0 * Constants.ShooterConstants.pivot_motor_multiplier);
      return;
    }
    if(speed < -1.0){
      pivot_motor.set(-1.0 * Constants.ShooterConstants.pivot_motor_multiplier);
      return;
    }
    pivot_motor.set(speed * Constants.ShooterConstants.pivot_motor_multiplier);
  }
}
