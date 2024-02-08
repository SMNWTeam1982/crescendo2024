package frc.robot.Subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Utilities.LimelightHelpers;

public class Limelight {

    public static boolean visible_speaker(){
        set_pipeline(Constants.speaker_tracking_channel);
        return visible_target();
    }

    public static boolean visible_target(){
        return LimelightHelpers.getTV("");
    }
    public static double get_x(){
        return LimelightHelpers.getTX("");
    }

    public static double get_y(){
        return LimelightHelpers.getTY("");
    }

    public static void set_pipeline(int pipeline_id){
        LimelightHelpers.setPipelineIndex("",pipeline_id);
    }

    public static Pose2d get_field_position(){
        set_pipeline(Constants.field_movement_channel);
        return LimelightHelpers.getBotPose3d("").toPose2d();
    }

    /**
     * 
     * @return
     */
    public static Pose2d get_positive_alliance_position(){
        Pose2d robot_pose = get_field_position();

        Translation2d new_position = new Translation2d(
            Math.abs(robot_pose.getX()),
            robot_pose.getY()
        );

        Rotation2d new_rotation;

        if(robot_pose.getX() >= 0.0){
            new_rotation = robot_pose.getRotation();
        }else{
            new_rotation = new Rotation2d(
                Math.abs(robot_pose.getRotation().getCos()),
                robot_pose.getRotation().getSin()
            );
        }

        return new Pose2d(
            new_position,
            new_rotation
        );
    }

    public static Rotation2d get_field_rotation(){
        return get_field_position().getRotation();
    }

}