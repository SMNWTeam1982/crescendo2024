// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Utilities.PID;
import edu.wpi.first.math.MathUtil;

public class Intake extends SubsystemBase {
  private final CANSparkMax pivot_motor;
  private final RelativeEncoder pivot_encoder;
  private final CANSparkMax intake_motor;
  private final PID pivot_pid = new PID(
    Constants.IntakeConstants.pivot_p,
    Constants.IntakeConstants.pivot_i,
    Constants.IntakeConstants.pivot_d
  );
  private Rotation2d target_angle = Constants.IntakeConstants.intake_starting_position;
  private boolean intake_deployed = false;
  /** Creates a new Intake. */
  public Intake(int pivot_motor_channel, int intake_motor_channel) {
    pivot_motor = new CANSparkMax(pivot_motor_channel, MotorType.kBrushless);
    pivot_encoder = pivot_motor.getEncoder();
    intake_motor = new CANSparkMax(intake_motor_channel, MotorType.kBrushless);
    pivot_encoder.setPosition(Constants.IntakeConstants.intake_starting_position.getRotations());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake error",target_angle.getDegrees() - get_intake_angle().getDegrees());
  }

  public Command get_intake_command(Supplier<Double> intake_function, Supplier<Boolean> toggle_deploy_function, Supplier<Boolean> zero_position_function){
    return run(
      () -> {
        set_intake(intake_function.get());

        if(toggle_deploy_function.get()){
          intake_deployed = !intake_deployed;
        }
        
        if(intake_deployed){
          //target_angle = Constants.IntakeConstants.deployed_angle.plus(Rotation2d.fromDegrees(shooter_controller.getRightY() * 5.0));
          target_angle = Constants.IntakeConstants.deployed_angle;
        } else{
          target_angle = Constants.IntakeConstants.handoff_angle;
        }

        set_pivot_motor(get_pid());

        if(zero_position_function.get()){

          double time = Timer.getFPGATimestamp();
          while(Timer.getFPGATimestamp() < time+1.0);
          pivot_encoder.setPosition(Constants.IntakeConstants.intake_starting_position.getRotations());
        }
      }
    );
  }

  public void set_intake(double speed){
    if(speed > 1.0){
      intake_motor.set(1.0 * Constants.IntakeConstants.intake_multiplier);
      return;
    }
    if(speed < -1.0){
      intake_motor.set(-1.0 * Constants.IntakeConstants.intake_multiplier);
    }
    intake_motor.set(speed * Constants.IntakeConstants.intake_multiplier);
  }

  public double get_pid(){
    double normal_target_angle = MathUtil.inputModulus(target_angle.getDegrees(), 0.0, 360.0);
    return pivot_pid.out(get_intake_angle().getDegrees(), normal_target_angle, 0.0);
  }

  public Rotation2d get_intake_angle(){

    return Rotation2d.fromDegrees(
      MathUtil.inputModulus(
        Rotation2d.fromRotations(pivot_encoder.getPosition() * Constants.IntakeConstants.pivot_motor_rotations_to_intake_rotations)
        .plus(Constants.IntakeConstants.intake_starting_position).getDegrees(),
        0.0,
        360.0
      )
    );
  }

  public void set_pivot_motor(double speed){
    if(speed > 1.0){
      pivot_motor.set(1.0 * Constants.IntakeConstants.pivot_motor_multiplier);
      return;
    }
    if(speed < -1.0){
      pivot_motor.set(-1.0 * Constants.IntakeConstants.pivot_motor_multiplier);
      return;
    }
    pivot_motor.set(speed * Constants.IntakeConstants.pivot_motor_multiplier);
  }
}
