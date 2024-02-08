package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static final double wheel_p = 0.038;
    public static final double wheel_i = 0.0;
    public static final double wheel_d = 0.00024;
    
    public static final Rotation2d front_right_encoder_offset = Rotation2d.fromDegrees(305.420);//33.419921875;//35.68359375;//34.365234375;//45.0;//34.365234375;
    public static final Rotation2d front_right_tangent = Rotation2d.fromDegrees(135.0);

    public static final Rotation2d front_left_encoder_offset = Rotation2d.fromDegrees(177.715 + 180.0);//89.9785150;//87.451171875;//93.275390625;//90.0;//93.275390625;
    public static final Rotation2d front_left_tangent = Rotation2d.fromDegrees(45.0);

    public static final Rotation2d back_left_encoder_offset = Rotation2d.fromDegrees(179.385 + 180.0);//87.802734375;//87.802734375;//87.978515625; //90.0;//87.978515625;
    public static final Rotation2d back_left_tangent = Rotation2d.fromDegrees(315.0);

    public static final Rotation2d back_right_encoder_offset = Rotation2d.fromDegrees(140.449);//231.416015625;//229.921875;//230.625;//45.0;//230.625;
    public static final Rotation2d back_right_tangent = Rotation2d.fromDegrees(225.0);

    public static final double wheel_rotations_to_meters_traveled = 1.0;

    //public static final double[] idHeights = {48.125, 48.125, 53.88, 53.88, 50.125, 50.125, 53.88, 53.88, 48.125, 48.125, 48.81, 48.81, 48.81, 48.81, 48.81, 48.81};

    public static final double limelightMountAngleDegrees = 5; 
    public static final double limelightLensHeightInches = 8.55; 

    public static final double target_tracking_p = -0.05;
    public static final double target_tracking_i = 0.0;
    public static final double target_tracking_d = 0.001;

    public static final double auto_move_p = 0.5;
    public static final double auto_move_i = 0.0;
    public static final double auto_move_d = 0.01;

    public static final double winchArmP = 0.035;
    public static final double winchArmI = 0.0;
    public static final double winchArmD = 0.0;
    public static final double extenLen = 0.0;

    public static final double pivotShooterP = 0.0;
    public static final double pivotShooterI = 0.0;
    public static final double pivotShooterD = 0.0;
    public static final double encoder_to_deg_shoot = 0.0;
    public static final double encoder_to_deg_in = 0.0;
    public static final double intake_deg_offset = 0.0;
    public static final double intake_deg_ground = 0.0;
    public static final double intake_deg_stop = 0.0;

    public static final double speaker_id_height = 51.875;
    public static final double speaker_height = 78.25;

    public static final double speed_dampen_multiplier = 0.4;//0.2
    public static final double rotational_dampen_multiplier = 0.15;//0.25

    public static final double angular_velocity_dampen_multiplier = 0.2;

    public static final int field_movement_channel = 0;
    public static final int speaker_tracking_channel = 1;
}
