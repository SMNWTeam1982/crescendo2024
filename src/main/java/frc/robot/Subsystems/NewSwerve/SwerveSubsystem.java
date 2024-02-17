// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.NewSwerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Limelight;
import frc.robot.Utilities.Tracking;

public class SwerveSubsystem extends SubsystemBase {
  private Gyroscope gyro = new Gyroscope();

  private final SwerveModule front_right_wheel = new SwerveModule(
    5,
    6,
    1,
    Constants.DriveConstants.front_right_encoder_offset
  );
  
  private final SwerveModule front_left_wheel = new SwerveModule(
    3,
    4,
    2,
    Constants.DriveConstants.front_left_encoder_offset
  );

  private final SwerveModule back_left_wheel = new SwerveModule(
    1,
    2,
    3,
    Constants.DriveConstants.back_left_encoder_offset
  );

  private final SwerveModule back_right_wheel = new SwerveModule(
    7,
    8,
    4,
    Constants.DriveConstants.back_right_encoder_offset
  );

  private final SwerveDrivePoseEstimator pose_estimator = new SwerveDrivePoseEstimator(
    Constants.DriveConstants.kinematics,
    get_gyro_angle(),
    get_module_positions(),
    Limelight.get_field_position()
  );


  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    set_move_braking(true);
  }

  public void set_turn_braking(boolean braking){
    front_right_wheel.set_turn_braking(braking);
    front_left_wheel.set_turn_braking(braking);
    back_left_wheel.set_turn_braking(braking);
    back_right_wheel.set_turn_braking(braking);
  }

  public void set_move_braking(boolean braking){
    front_right_wheel.set_move_braking(braking);
    front_left_wheel.set_move_braking(braking);
    back_left_wheel.set_move_braking(braking);
    back_right_wheel.set_move_braking(braking);
  }

  public void set_gyro_offset(Rotation2d offset){
    gyro.set_offset(offset);
  }

  public Rotation2d get_gyro_angle(){
    return gyro.get_angle();
  }

  public SwerveModuleState[] get_module_states(){
    return new SwerveModuleState[] {
      front_right_wheel.get_state(),
      front_left_wheel .get_state(),
      back_left_wheel  .get_state(),
      back_right_wheel .get_state()
    };
  }
  
  public SwerveModulePosition[] get_module_positions(){
    return new SwerveModulePosition[] {
      front_right_wheel.get_position(),
      front_left_wheel .get_position(),
      back_left_wheel  .get_position(),
      back_right_wheel .get_position()
    };
  }

  public void reset_pose(Pose2d pose){
    pose_estimator.resetPosition(get_gyro_angle(), get_module_positions(), pose);
  }

  public void set_chassis_speed(ChassisSpeeds robot_relative_speeds){
    SwerveModuleState[] module_states = DriveConstants.kinematics.toSwerveModuleStates(robot_relative_speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(module_states, DriveConstants.wheel_max_speed_meters_per_second);
    front_right_wheel.set_desired_state(module_states[0]);
    front_left_wheel.set_desired_state(module_states[1]);
    back_left_wheel.set_desired_state(module_states[2]);
    back_right_wheel.set_desired_state(module_states[3]);
  }

  public Pose2d get_robot_pose(){
    return pose_estimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Limelight.visible_tracking_target()){
      Tracking limelight_results = Limelight.get_field_tracking();
      pose_estimator.addVisionMeasurement(limelight_results.pose, limelight_results.timestamp, limelight_results.standard_deviation);
    }
    pose_estimator.update(get_gyro_angle(), get_module_positions());
  }
}
