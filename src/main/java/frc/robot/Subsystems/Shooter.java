// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.PID;
import frc.robot.Utilities.Rotation2dFix;

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
  private final PID motor_velocity_pid = new PID(
    Constants.ShooterConstants.shoot_speed_p,
    Constants.ShooterConstants.shoot_speed_i,
    Constants.ShooterConstants.shoot_speed_d
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
    pivot_encoder.setPosition(
      Rotation2dFix.fix(Constants.ShooterConstants.shooter_start_angle).getRotations()
      /Constants.ShooterConstants.pivot_motor_rotations_to_shooter_rotations
    );

    pivot_motor.setIdleMode(IdleMode.kCoast);
    pivot_motor.setIdleMode(IdleMode.kCoast);

    setDefaultCommand(idle_command());
  }

  public Command shooter_command(Supplier<Boolean> shoot_function, Supplier<Double> shoot_amount_function, Supplier<Rotation2d> target_angle_function){
    return run(
      () -> {
        if(shoot_function.get()){
          set_shoot_motors(shoot_amount_function.get());
          
        }else{
          set_shoot_motors(0.0);
        }

        SmartDashboard.putNumber("shoot power", shoot_amount_function.get());


        target_angle = target_angle_function.get();

        set_pivot_motor(get_pid());
      }
    );
  }

  public Command auto_aim_command(Pose2d robot_pose, boolean is_on_red_side){
    return runOnce(
      () -> {
        Translation2d shooter_pivot_position = robot_pose.getTranslation().plus(
          new Translation2d(Constants.ShooterConstants.shooter_pivot_point_forward,robot_pose.getRotation())
        );

        double dist_meters = 0.0;
        if(is_on_red_side){
          dist_meters = shooter_pivot_position.getDistance(Constants.ShooterConstants.red_speaker_position);
        }else{
          dist_meters = shooter_pivot_position.getDistance(Constants.ShooterConstants.blue_speaker_position);
        }

        target_angle = Rotation2d.fromRadians(Math.atan(Constants.ShooterConstants.speaker_height_difference / dist_meters));
      }
    );
  }

  public Command idle_command(){
    return runOnce(
      () -> {
        set_pivot_motor(get_pid());
      }
    );
  }

  public Command start_motors_command(){
    return runOnce(
      () -> set_shoot_motors(0.75)
    );
  }

  public Command stop_motors_command(){
    return runOnce(
      () -> set_shoot_motors(0.0)
    );
  }

  public Command set_angle_command(Rotation2d angle){
    return runOnce(
      () -> {target_angle = angle;}
    );
  }

  public Command get_calibrate_command(){
    return runOnce(
      () -> {
        double time = Timer.getFPGATimestamp();
        while(Timer.getFPGATimestamp() < time+1.0){};
        pivot_encoder.setPosition(Constants.ShooterConstants.shooter_start_angle.getRotations());
      }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("shooter angle",get_shooter_angle().getDegrees());
    // SmartDashboard.putNumber("target shooter angle", target_angle.getDegrees());
    // SmartDashboard.putNumber("upper speed", upper_shoot_motor.getEncoder().getVelocity() / Constants.ShooterConstants.shoot_motor_max_rpm);
    // SmartDashboard.putNumber("lower speed", lower_shoot_motor.getEncoder().getVelocity() / Constants.ShooterConstants.shoot_motor_max_rpm);

    SmartDashboard.putBoolean("up to speed", upper_shoot_motor.getEncoder().getVelocity() / Constants.ShooterConstants.shoot_motor_max_rpm >= 0.6);

    SmartDashboard.putNumber("angle", get_shooter_angle().getDegrees());
    SmartDashboard.putNumber("target", target_angle.getDegrees());
  }

  public double get_pid(){
    return pivot_pid.out(get_shooter_angle().getDegrees(), target_angle.getDegrees(), 0.0);
  }

  public Rotation2d get_shooter_angle(){
    return Rotation2dFix.fix(
      Rotation2d.fromRotations(pivot_encoder.getPosition() * Constants.ShooterConstants.pivot_motor_rotations_to_shooter_rotations)
    );
  }

  public void set_shoot_motors(double speed){

    double desired_upper_shooter_speed = speed + motor_velocity_pid.out(upper_shoot_motor.getEncoder().getVelocity() / Constants.ShooterConstants.shoot_motor_max_rpm, speed ,0.0);

    double upper_speed = MathUtil.clamp(
      desired_upper_shooter_speed,
      -1.0,
      1.0
    );

    double desired_lower_shooter_speed = -speed - motor_velocity_pid.out(lower_shoot_motor.getEncoder().getVelocity() / Constants.ShooterConstants.shoot_motor_max_rpm, -speed ,0.0);

    

    double lower_speed = MathUtil.clamp(
      desired_lower_shooter_speed,
      -1.0,
      1.0
    );


    

    upper_shoot_motor.set(speed * Constants.ShooterConstants.upper_shoot_motor_multiplier);
    lower_shoot_motor.set(-speed * Constants.ShooterConstants.lower_shoot_motor_multiplier);
  }

  public void set_pivot_motor(double speed){
    pivot_motor.set(MathUtil.clamp(speed, -1.0, 1.0) * Constants.ShooterConstants.pivot_motor_multiplier);
  }
}
