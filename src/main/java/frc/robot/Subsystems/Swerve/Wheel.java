package frc.robot.Subsystems.Swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Utilities.PID;
import frc.robot.Utilities.PolarVector;
import frc.robot.Utilities.Rotation2dFix;
import frc.robot.Utilities.Vector;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;



public class Wheel {
    public final CANSparkMax go_motor;
    public final RelativeEncoder go_motor_encoder;
    public final CANSparkMax turn_motor;
    public final CANcoder encoder;
    public final Rotation2d encoder_offset;
    public final Rotation2d tangent_angle;
    private final PID wheel_rotation_pid = new PID(
        Constants.DriveConstants.wheel_rotation_p,
        Constants.DriveConstants.wheel_rotation_i,
        Constants.DriveConstants.wheel_rotation_d
    );

    //private final SimpleMotorFeedforward feeder = new SimpleMotorFeedforward()

    private Rotation2d last_rotation = Rotation2d.fromDegrees(0.0);

    public Wheel(int go_motor_channel, int turn_motor_channel, int encoder_channel, Rotation2d encoder_offset, Rotation2d tangent_angle){
        go_motor = new CANSparkMax(go_motor_channel, MotorType.kBrushless);
        go_motor_encoder = go_motor.getEncoder();
        turn_motor = new CANSparkMax(turn_motor_channel, MotorType.kBrushless);
        encoder = new CANcoder(encoder_channel);
        this.encoder_offset = encoder_offset;
        this.tangent_angle = tangent_angle;

        zero_wheel_position();

        go_motor.restoreFactoryDefaults();
        // go_motor.setClosedLoopRampRate(0.1);
        // go_motor.setOpenLoopRampRate(0.1);
        turn_motor.restoreFactoryDefaults();

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoder.getConfigurator().apply(config);
    }

    public SwerveModuleState get_state(){
        return new SwerveModuleState(
            get_wheel_velocity(),
            get_encoder()
        );
    }
    public SwerveModulePosition get_position(){
        return new SwerveModulePosition(
            get_wheel_position(),
            get_encoder()
        );
    }

    public double get_wheel_velocity(){
        return go_motor_encoder.getVelocity() * Constants.DriveConstants.go_motor_rpm_to_meters_per_second;
    }
    public double get_wheel_position(){
        return go_motor_encoder.getPosition() * Constants.DriveConstants.motor_rotations_to_meters_traveled;
    }
    public void zero_wheel_position(){
        go_motor_encoder.setPosition(0.0);
    }

    public Rotation2d get_encoder(){
        return Rotation2dFix.fix(Rotation2d.fromDegrees(encoder.getAbsolutePosition().getValueAsDouble() * 360.0).minus(encoder_offset));
    }

    public double get_wheel_rotation_PID(Rotation2d angle){
        return wheel_rotation_pid.out(angle.getDegrees(), 0,0);
    }

    public void set_turn_braking(boolean brake){
        if(brake){
            turn_motor.setIdleMode(IdleMode.kBrake);
        }else{
            turn_motor.setIdleMode(IdleMode.kCoast);
        }
    }

    public void set_move_braking(boolean brake){
        if(brake){
            go_motor.setIdleMode(IdleMode.kBrake);
        }else{
            go_motor.setIdleMode(IdleMode.kCoast);
        }
    }
    
    /**
     * 
     * @param desired_robot_velocity the robot oriented linear velocity arbitrary units
     * @param angular_velocity -1 to 1 arbitrary units
     * @return the desired movement for the wheel
     */
    public PolarVector calculate( PolarVector desired_robot_velocity, double angular_velocity){

        PolarVector tangent_vector = new PolarVector(tangent_angle.unaryMinus(), angular_velocity * Constants.DriveConstants.angular_velocity_multiplier);

        Vector result_vector = new Vector(
            tangent_vector.x() + desired_robot_velocity.x(),
            tangent_vector.y() + desired_robot_velocity.y()
        );

        return new PolarVector(result_vector.angle(), result_vector.length() / 2.0);
    }

    public void run( PolarVector desired_movement, double velocity_multiplier){
        if(desired_movement.length == 0.0){
            move_and_pivot(new PolarVector(last_rotation, 0.0));
        }
        
        move_and_pivot(
            new PolarVector(
                desired_movement.angle,
                desired_movement.length * velocity_multiplier
            )
        );

        last_rotation = desired_movement.angle;
    }

    /**
     * https://www.desmos.com/calculator/obxtmgen7n
     * @param target_angle
     * @param speed
     */
    public void move_and_pivot(PolarVector desired_state){
        // see link below for visual representation
        // https://www.desmos.com/calculator/obxtmgen7n 
        
    
        Rotation2d current_angle = get_encoder();
        Rotation2d perpendicular_angle = Rotation2dFix.fix(current_angle.plus(Rotation2d.fromDegrees(90.0)));
        Rotation2d opposite_angle = Rotation2dFix.fix(current_angle.plus(Rotation2d.fromDegrees(180.0)));
    
        // establishes the three angles used to determine the optimal way to move the wheel
        Rotation2d delta = angle_between( current_angle , desired_state.angle );
        Rotation2d zeta = angle_between( perpendicular_angle , desired_state.angle );
        Rotation2d gamma = angle_between( opposite_angle , desired_state.angle );
        if (gamma.getDegrees() < delta.getDegrees()){ // checks for the smallest distance to the target angle

            double power = get_wheel_rotation_PID( gamma ); // minimize gamma
            
            if (zeta.getDegrees() > 90.0){ // Checks if left or right movement is needed
                move_turn_motor( power );
            }else{
                move_turn_motor( -power );
            }
            
            move_go_motor( -desired_state.length ); // forwards is backwards
        }else{
            
            double power = get_wheel_rotation_PID( delta ); // minimize delta
    
            if (zeta.getDegrees() > 90.0){ // Checks if left or right movement is needed
                move_turn_motor( -power );
            }else{
                move_turn_motor( power );
            }
    
            move_go_motor( desired_state.length ); // forwards is forwards
        }
    }
    
    // uses dot product formula to find the angle between the two angles
    public static Rotation2d angle_between( Rotation2d angle_1 , Rotation2d angle_2 ){
        return Rotation2dFix.fix(
            Rotation2d.fromRadians(
                Math.acos(
                    angle_1.getCos() * angle_2.getCos() + 
                    angle_1.getSin() * angle_2.getSin()
                )
            )
        );
    }

    public void move_turn_motor( double speed ){
        turn_motor.set( MathUtil.clamp(speed, -1.0, 1.0) * Constants.DriveConstants.wheel_rotation_multiplier );
        //turn_motor.getOutputCurrent()
    }

    public void move_go_motor( double speed ){
        
        go_motor.set( MathUtil.clamp(speed, -1.0, 1.0) * Constants.DriveConstants.wheel_speed_multiplier); 
        //SmartDashboard.putNumber(String.valueOf(encoder.getDeviceID()), MathUtil.clamp(speed, -1.0, 1.0) * Constants.DriveConstants.wheel_speed_multiplier);
    }
}