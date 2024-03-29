package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public static final class DriveConstants {
        
        public static final double wheel_speed_multiplier = 0.4;//0.2
        public static final double wheel_rotation_multiplier = 0.25;//0.25
        public static final double angular_velocity_multiplier = 0.4;

        public static final double wheel_rotation_p = 0.038;
        public static final double wheel_rotation_i = 0.0;
        public static final double wheel_rotation_d = 0.00024;

        public static final double note_rotation_p = 0.0;
        public static final double note_rotation_i = 0.0;
        public static final double note_rotation_d = 0.0;

        public static final double note_movement_p = 0.0;
        public static final double note_movement_i = 0.0;
        public static final double note_movement_d = 0.0;
        
        public static final Rotation2d front_right_encoder_offset = Rotation2d.fromDegrees(305.420);//33.419921875;//35.68359375;//34.365234375;//45.0;//34.365234375;

        public static final Rotation2d front_left_encoder_offset = Rotation2d.fromDegrees(177.715 + 180.0);//89.9785150;//87.451171875;//93.275390625;//90.0;//93.275390625;

        public static final Rotation2d back_left_encoder_offset = Rotation2d.fromDegrees(179.385 + 180.0);//87.802734375;//87.802734375;//87.978515625; //90.0;//87.978515625;

        public static final Rotation2d back_right_encoder_offset = Rotation2d.fromDegrees(140.449);//231.416015625;//229.921875;//230.625;//45.0;//230.625;

        // 6.75 rotations of the motor = 1 rotation of the wheel
        // wheel diameter is about 4 inches
        // measured wheel circimference is 28.55cm or 0.2855m
        // 1 rpm = 7.049382716E-4
        public static final double wheel_rotations_to_meters_traveled = 0.2855;
        public static final double go_motor_rpm_to_meters_per_second = 7.049382716E-4;

        public static final double wheel_max_speed_meters_per_second = 1.0;

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(0.2635,-0.2635),
            new Translation2d(0.2635,0.2635),
            new Translation2d(-0.2635,0.2635),
            new Translation2d(-0.2635,-0.2635)
        );
    }

    public static final class AutoConstants{
        public static final double target_tracking_p = -0.05;
        public static final double target_tracking_i = 0.0;
        public static final double target_tracking_d = 0.001;

        public static final double auto_move_p = 0.5;
        public static final double auto_move_i = 0.0;
        public static final double auto_move_d = 0.01;
    }

    public static final class LimeLightConstants{
        public static final double speaker_angle_multiplier = 0.8;
        public static final int field_movement_channel = 0;
        public static final int speaker_tracking_channel = 1;
        public static final int speaker_aiming_channel = 2;
    }

    public static final class ShooterConstants{
        public static final double pivot_p = 0.015;
        public static final double pivot_i = 0.0;
        public static final double pivot_d = 0.0;
        public static final Rotation2d shooter_start_angle = Rotation2d.fromDegrees(68.0);
        
        public static final Rotation2d shooter_load_angle = Rotation2d.fromDegrees(45.0);
        // gear ratio is 100:1
        public static final double pivot_motor_rotations_to_shooter_rotations = 0.01;

        public static final double pivot_motor_multiplier = 1.0;

        public static final double upper_shoot_motor_multiplier = 1.0;
        public static final double lower_shoot_motor_multiplier = -1.0;

        public static final double shooter_pivot_point_height = 0.46; // meters

        public static final double shooter_pivot_position_relative_to_center = 0.28; // meters
    }

    public static final class IntakeConstants{
        public static final double pivot_p = 0.2;
        public static final double pivot_i = 0.0;
        public static final double pivot_d = 0.0;
        public static final Rotation2d intake_starting_position = Rotation2d.fromDegrees(10.0);
        //gear ratio is 80:1
        public static final double pivot_motor_rotations_to_intake_rotations = 0.0125;

        public static final double pivot_motor_multiplier = 0.3;
        public static final double intake_multiplier = 0.5;

        public static final Rotation2d deployed_angle = Rotation2d.fromDegrees(210.0);
        public static final Rotation2d handoff_angle = Rotation2d.fromDegrees(25.0);
    }
    public static final double speaker_id_height = 51.875;
    public static final double speaker_height = 78.25;
}
