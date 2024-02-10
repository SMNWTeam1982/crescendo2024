package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Limelight;
import frc.robot.Utilities.PolarVector;
import frc.robot.Utilities.Tracking;

public class Drive extends SubsystemBase {
    private Gyroscope gyro = new Gyroscope();

    private final Wheel front_right_wheel = new Wheel(
        5,
        6,
        1,
        Constants.DriveConstants.front_right_encoder_offset,
        Constants.DriveConstants.front_right_tangent
    );
    
    private final Wheel front_left_wheel = new Wheel(
        3,
        4,
        2,
        Constants.DriveConstants.front_left_encoder_offset,
        Constants.DriveConstants.front_left_tangent
    );

    private final Wheel back_left_wheel = new Wheel(
        1,
        2,
        3,
        Constants.DriveConstants.back_left_encoder_offset,
        Constants.DriveConstants.back_left_tangent
    );

    private final Wheel back_right_wheel = new Wheel(
        7,
        8,
        4,
        Constants.DriveConstants.back_right_encoder_offset,
        Constants.DriveConstants.back_right_tangent
    );

    private final SwerveDrivePoseEstimator pose_estimator = new SwerveDrivePoseEstimator(
        Constants.DriveConstants.kinematics,
        get_gyro_angle(),
        get_module_positions(),
        Limelight.get_field_position()
    );

    public Drive() {}

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

    /**
     * @param desired_robot_linear_velocity
     * @param desired_angular_velocity
     * @return the wheels desired output in order of front_right, front_left, back_left, back_right
     */
    public PolarVector[] calculate_wheel_velocities(PolarVector desired_robot_linear_velocity, double desired_angular_velocity){
        return new PolarVector[] {
            front_right_wheel.calculate(desired_robot_linear_velocity, desired_angular_velocity),
            front_left_wheel .calculate(desired_robot_linear_velocity, desired_angular_velocity),
            back_left_wheel  .calculate(desired_robot_linear_velocity, desired_angular_velocity),
            back_right_wheel .calculate(desired_robot_linear_velocity, desired_angular_velocity)
        };
    }

    public void run_wheels(PolarVector[] desired_wheel_velocities, double velocity_multiplier){
        front_right_wheel.run(desired_wheel_velocities[0], velocity_multiplier);
        front_left_wheel .run(desired_wheel_velocities[1], velocity_multiplier);
        back_left_wheel  .run(desired_wheel_velocities[2], velocity_multiplier);
        back_right_wheel .run(desired_wheel_velocities[3], velocity_multiplier);
    }

    public SwerveModuleState[] get_module_states(){
        return new SwerveModuleState[] {
            front_right_wheel.get_state(),
            front_left_wheel .get_state(),
            back_left_wheel  .get_state(),
            back_right_wheel .get_state(),
        };
    }
    
    public SwerveModulePosition[] get_module_positions(){
        return new SwerveModulePosition[] {
            front_right_wheel.get_position(),
            front_left_wheel .get_position(),
            back_left_wheel  .get_position(),
            back_right_wheel .get_position(),
        };
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

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
