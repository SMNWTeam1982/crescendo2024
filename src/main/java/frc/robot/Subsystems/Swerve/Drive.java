package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Limelight;
import frc.robot.Utilities.PID;
import frc.robot.Utilities.PolarVector;
import frc.robot.Utilities.Rotation2dFix;
import frc.robot.Utilities.Tracking;
import frc.robot.Utilities.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.PIDConstants;


public class Drive extends SubsystemBase {
    private Gyroscope gyro = new Gyroscope();

    private boolean is_on_red_side = true;

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
        get_field_angle(),
        get_module_positions(),
        Limelight.get_field_position()
    );

    private PolarVector[] last_velocities;

    private final PID robot_rotation_pid = new PID(
        Constants.DriveConstants.robot_rotation_p,
        Constants.DriveConstants.robot_rotation_i,
        Constants.DriveConstants.robot_rotation_d
    );

    private final PID robot_antidrift_pid = new PID(
        Constants.DriveConstants.antidrift_p,
        Constants.DriveConstants.antidrift_i,
        Constants.DriveConstants.antidrift_d
    );

    private Rotation2d target_angle = Rotation2d.fromDegrees(0.0);

    //private final Field2d field = new Field2d();

    public Drive() {
        //SmartDashboard.putData("field", field);
        AutoBuilder.configureHolonomic(
            this::getEstimatedPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive_from_chassis_speeds,  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(1.0, 0, 0), // Translation PID constants
                new PIDConstants(1.0, 0, 0), // Rotation PID constants
                Constants.DriveConstants.max_speed_meters_per_second, // Max module speed, in m/s
                0.47, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig()
            ), 
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public Pose2d getEstimatedPose(){
        return pose_estimator.getEstimatedPosition();
    }

    public boolean attempt_set_team(){
        if(DriverStation.getAlliance().isEmpty()){
            return false;
        }
    
        if(DriverStation.getAlliance().get() == Alliance.Red){
            is_on_red_side = true;
        }else{
            is_on_red_side = false;
        };

        return true;
    }

    public boolean is_on_red_side(){
        return is_on_red_side;
    }

    public double get_rotation_pid(Rotation2d desired_angle){
        //stolen from wheel logic
        Rotation2d current_angle = get_field_angle();
        Rotation2d perpendicular_angle = Rotation2dFix.fix(current_angle.plus(Rotation2d.fromDegrees(90.0)));
    
        // establishes the three angles used to determine the optimal way to move the wheel
        Rotation2d delta = Wheel.angle_between( current_angle , desired_angle );
        Rotation2d zeta = Wheel.angle_between( perpendicular_angle , desired_angle );

        double power = MathUtil.clamp(robot_rotation_pid.out(delta.getDegrees(),0.0,0.0),-1.0,1.0);
        
        if (zeta.getDegrees() > 90.0){ // Checks if left or right movement is needed
            return power;
        }else{
            return -power;
        }
    }

    public double get_rotation_pid_antidrift(Rotation2d desired_angle){
        //stolen from wheel logic
        Rotation2d current_angle = get_field_angle();
        Rotation2d perpendicular_angle = Rotation2dFix.fix(current_angle.plus(Rotation2d.fromDegrees(90.0)));
    
        // establishes the three angles used to determine the optimal way to move the wheel
        Rotation2d delta = Wheel.angle_between( current_angle , desired_angle );
        Rotation2d zeta = Wheel.angle_between( perpendicular_angle , desired_angle );

        double power = MathUtil.clamp(robot_antidrift_pid.out(delta.getDegrees(),0.0,0.0),-1.0,1.0);
        
        if (zeta.getDegrees() > 90.0){ // Checks if left or right movement is needed
            return power;
        }else{
            return -power;
        }
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

    public void zero_gyro(){
        gyro.zero_angle();
    }

    public Rotation2d get_field_angle(){
        return gyro.get_angle();
    }

    public Rotation2d get_driver_angle(){
        if(is_on_red_side){
            return gyro.get_red_angle();
        }else{
            return gyro.get_blue_angle();
        }
    }

    public boolean attempt_field_orient(){
        boolean visible_target = Limelight.visible_tracking_target();
        if(visible_target){
            Tracking limelight_results = Limelight.get_field_tracking();
            gyro.field_orient(limelight_results.pose.getRotation());
        }
        return visible_target;
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
    
    public void drive_from_chassis_speeds(ChassisSpeeds speeds){
        double x = MathUtil.clamp(speeds.vxMetersPerSecond / Constants.DriveConstants.max_speed_meters_per_second, -1.0,1.0);
        double y = MathUtil.clamp(speeds.vyMetersPerSecond / Constants.DriveConstants.max_speed_meters_per_second, -1.0,1.0);

        double speed = Math.sqrt(x * x + y * y);
        if(speed > 1.0){speed = 1.0;}
        if(speed < 0.1){speed = 0.0;}

        PolarVector desired_velocity = new PolarVector(
            Rotation2d.fromRadians(Math.atan2(y,x)).plus(Rotation2d.fromDegrees(90.0)),
            speed
        );

        PolarVector[] wheel_velocities = calculate_wheel_velocities(desired_velocity, speeds.omegaRadiansPerSecond/Constants.DriveConstants.max_radians_per_second);
  
        double highest = 0.0;
        for(PolarVector wheel_velocity : wheel_velocities){
          if( wheel_velocity.length > highest ){
            highest = wheel_velocity.length;
          }
        }
  
        double multiplier = MathUtil.clamp(1.0 / highest,0.0,1.0);
  
        if(highest < 0.01){
            multiplier = 1.0;
        }
        
        run_wheels(wheel_velocities, multiplier);
    }

    private void resetPose(Pose2d pos){}

    public void BuilderConfigure(){
        
    }

    public Command go_in_direction_command(PolarVector direction){
        return runOnce(
            () -> {
                PolarVector field_velocity = new PolarVector(direction.angle.minus(get_driver_angle()), direction.length);
                PolarVector[] velocities = calculate_wheel_velocities(field_velocity, 0.0);
                last_velocities = velocities;
                run_wheels(velocities, 1.0);
            }
        );
    }

    public Command stop_command(){
        return runOnce(
            () -> stop_wheels()
        );
    }

    public Command idle_command(){
        return runOnce(
            () -> {
                if(last_velocities != null){
                    run_wheels(last_velocities, 1.0);
                }

                // if(Limelight.visible_tracking_target()){
                //     Tracking limelight_results = Limelight.get_field_tracking();
                //     pose_estimator.addVisionMeasurement(
                //         limelight_results.pose,
                //         limelight_results.timestamp,
                //         limelight_results.standard_deviation
                //     );
                // }

                pose_estimator.update(get_field_angle(), get_module_positions());

                //SmartDashboard.putString("pose",get_robot_pose().toString());
            }
        );
    }

    public void run_wheels(PolarVector[] desired_wheel_velocities, double velocity_multiplier){
        front_right_wheel.run(desired_wheel_velocities[0], velocity_multiplier);
        front_left_wheel .run(desired_wheel_velocities[1], velocity_multiplier);
        back_left_wheel  .run(desired_wheel_velocities[2], velocity_multiplier);
        back_right_wheel .run(desired_wheel_velocities[3], velocity_multiplier);
    }

    public void stop_wheels(){
        last_velocities = new PolarVector[] {
            new PolarVector(new Rotation2d(), 0.0),
            new PolarVector(new Rotation2d(), 0.0),
            new PolarVector(new Rotation2d(), 0.0),
            new PolarVector(new Rotation2d(), 0.0)
        };

        front_right_wheel.move_go_motor(0.0);
        front_left_wheel.move_go_motor(0.0);
        back_left_wheel.move_go_motor(0.0);
        back_right_wheel.move_go_motor(0.0);
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

    public ChassisSpeeds getChassisSpeed() {
       return Constants.DriveConstants.kinematics.toChassisSpeeds(
            front_right_wheel.get_state(),
            front_left_wheel .get_state(),
            back_left_wheel  .get_state(),
            back_right_wheel .get_state()
        );
    }

    public Pose2d get_robot_pose(){

        //field.setRobotPose(get_robot_pose());

        return pose_estimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        if(Limelight.visible_tracking_target()){
            Tracking limelight_results = Limelight.get_field_tracking();
            pose_estimator.addVisionMeasurement(limelight_results.pose, limelight_results.timestamp, limelight_results.standard_deviation);
        }

        pose_estimator.update(get_field_angle(), get_module_positions());
        
        // SmartDashboard.putString("pose",get_robot_pose().toString());

        // SmartDashboard.putBoolean("is on red team",is_on_red_side());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
