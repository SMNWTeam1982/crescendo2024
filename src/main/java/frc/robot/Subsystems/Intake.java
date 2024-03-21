// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Utilities.PID;
import frc.robot.Utilities.Rotation2dFix;
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
  private double target_angle = 10.0;
  private boolean manually_overridden = false;

  private final DigitalInput note_detector;
  /** Creates a new Intake. */
  public Intake(int pivot_motor_channel, int intake_motor_channel, int note_detector_channel) {
    pivot_motor = new CANSparkMax(pivot_motor_channel, MotorType.kBrushless);
    pivot_encoder = pivot_motor.getEncoder();
    
    pivot_encoder.setPosition(
      (Constants.IntakeConstants.intake_starting_position/360.0)
      /Constants.IntakeConstants.pivot_motor_rotations_to_intake_rotations
    );
    intake_motor = new CANSparkMax(intake_motor_channel, MotorType.kBrushless);
    note_detector = new DigitalInput(note_detector_channel);
    pivot_motor.setIdleMode(IdleMode.kCoast);

    setDefaultCommand(idle_command());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("DIO, holding note", holding_note());
    SmartDashboard.putNumber("intake angle", get_intake_angle());
    SmartDashboard.putNumber("intake targt", target_angle);
  }

  public Command get_calibrate_command(){
    return runOnce(
      () -> {
        pivot_motor.setIdleMode(IdleMode.kCoast);
        double time = Timer.getFPGATimestamp();
        while(Timer.getFPGATimestamp() < time+1.0){};
        pivot_encoder.setPosition(MathUtil.inputModulus(Constants.IntakeConstants.intake_starting_position/360.0,0.0,1.0));
      }
    );
  }

  public boolean holding_note(){
    return note_detector.get();
  }

  public Command idle_command(){
    return run(
      () -> {
        set_pivot_motor(get_pid());
        if(holding_note()){
          intake_motor.set(0.0);
        }
      }
    );
  }

  public Command grab_note_command(){
    return new SequentialCommandGroup(
      deploy_command(),
      suck_command(),
      new ParallelCommandGroup(
        new WaitUntilCommand(
          () -> holding_note()
        ),
        idle_command()
      ),
      stop_command(),
      handoff_command(),
      new ParallelCommandGroup(
        new WaitCommand(1.0),
        idle_command()
      )
    );
  }

  public Command set_angle_command(double angle){
    return runOnce(
      () -> {
        if(angle > 0.0){
          target_angle = angle;
        }
      }
    );
  }

  public Command handoff_command(){
    return runOnce(
      () -> target_angle = Constants.IntakeConstants.handoff_angle
    );
  }

  public Command deploy_command(){
    return runOnce(
      () -> target_angle = Constants.IntakeConstants.deployed_angle
    );
  }

  public Command set_speed_command(double amount){
    return runOnce(
      () -> {
        if(holding_note() && amount > 0.0){
          set_intake(0.0);
        }else{
          set_intake(amount);
        }
      }
    );
  }

  public Command stop_command(){
    return runOnce(
      () -> set_intake(0.0)
    );
  }

  public Command eject_command(){
    return new SequentialCommandGroup(
      runOnce(
        () -> set_intake(-1.0)
      ),
      new WaitCommand(0.2)
    );
    
  }

  public Command suck_command(){
    return runOnce(
      () -> {
        set_intake(1.0);
      }
    );
  }

  public Command get_intake_command(
    Supplier<Double> intake_function,
    Supplier<Double> desired_angle_function, 
    Supplier<Boolean> toggle_manual_override_function,
    Supplier<Double> manual_movement_function,
    Supplier<Boolean> calibrate_function
  ){
    //pivot_encoder.setPosition(Constants.IntakeConstants.intake_starting_position.getRotations());
    return run(
      () -> {
        double intake_speed = intake_function.get();
        if(holding_note() && intake_speed > 0.0){
          intake_speed = 0.0;
        }
        set_intake(intake_speed);

        if(calibrate_function.get()){
          manually_overridden = false;
          pivot_encoder.setPosition(
            (Constants.IntakeConstants.intake_starting_position/360.0)
            /Constants.IntakeConstants.pivot_motor_rotations_to_intake_rotations
          );
        }

        if(toggle_manual_override_function.get()){
          manually_overridden = !manually_overridden;
        }

        if( manually_overridden ){
          set_pivot_motor(manual_movement_function.get());
        }else{
          double angle = desired_angle_function.get();

          if(angle > 0.0){
            target_angle = angle;
          }

          set_pivot_motor(get_pid());
        }


      }
    );
  }

  public void set_intake(double speed){
    intake_motor.set(MathUtil.clamp(speed, -1.0, 1.0) * Constants.IntakeConstants.intake_multiplier);
  }

  public double get_pid(){
    return pivot_pid.out(get_intake_angle(), target_angle, 0.0);
  }

  public double get_intake_angle(){
    return (pivot_encoder.getPosition() * Constants.IntakeConstants.pivot_motor_rotations_to_intake_rotations) * 360;
    // return Rotation2dFix.fix(
    //   Rotation2d.fromRotations(MathUtil.clamp(, 0.0, 359.0))
    //   .minus(offset).plus(Constants.IntakeConstants.intake_starting_position)
    // );
  }

  public void set_pivot_motor(double speed){
    // if(speed < 0.0 && get_intake_angle().getDegrees() < Constants.IntakeConstants.intake_starting_position.getDegrees()){
    //   pivot_motor.set(0.0);
    // }
    pivot_motor.set(MathUtil.clamp(speed, -1.0, 1.0) * Constants.IntakeConstants.pivot_motor_multiplier);
  }
}