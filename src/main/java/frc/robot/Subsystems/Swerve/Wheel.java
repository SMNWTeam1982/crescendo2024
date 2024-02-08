package frc.robot.Subsystems.Swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Utilities.PID;
import frc.robot.Utilities.PolarVector;
import frc.robot.Utilities.Vector;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;



public class Wheel {
    public CANSparkMax move_wheel;
    public CANSparkMax turn_wheel;
    public CANcoder encoder;
    public Rotation2d encoder_offset;
    public Rotation2d tangent_angle;
    private PID pid = new PID(Constants.wheel_p,Constants.wheel_i,Constants.wheel_d);

    private Rotation2d last_rotation = Rotation2d.fromDegrees(0.0);

    public Wheel(CANSparkMax move_wheel, CANSparkMax turn_wheel, CANcoder encoder, Rotation2d encoder_offset, Rotation2d tangent_angle){
        this.move_wheel = move_wheel;
        this.turn_wheel = turn_wheel;
        this.encoder = encoder;
        this.encoder_offset = encoder_offset;
        this.tangent_angle = tangent_angle;

        move_wheel.restoreFactoryDefaults();
        move_wheel.setClosedLoopRampRate(1.0);
        move_wheel.setOpenLoopRampRate(0.5);
        turn_wheel.restoreFactoryDefaults();

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoder.getConfigurator().apply(config);
    }

    public Rotation2d get_encoder(){
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition().getValueAsDouble() * 360.0).minus(encoder_offset); // degrees
    }

    public double get_PID(Rotation2d angle){
        return pid.Out(angle.getDegrees(), 0,0);
    }

    public void set_turn_braking(boolean brake){
        if(brake){
            turn_wheel.setIdleMode(IdleMode.kBrake);
        }else{
            turn_wheel.setIdleMode(IdleMode.kCoast);
        }
    }

    public void set_move_braking(boolean brake){
        if(brake){
            move_wheel.setIdleMode(IdleMode.kBrake);
        }else{
            move_wheel.setIdleMode(IdleMode.kCoast);
        }
    }
    
    /**
     * 
     * @param desired_robot_velocity the robot oriented linear velocity arbitrary units
     * @param angular_velocity -1 to 1 arbitrary units
     * @return the desired movement for the wheel
     */
    public PolarVector calculate( PolarVector desired_robot_velocity, double angular_velocity){

        PolarVector tangent_vector = new PolarVector(tangent_angle.unaryMinus(), angular_velocity * Constants.angular_velocity_dampen_multiplier);

        Vector result_vector = new Vector(
            tangent_vector.x() + desired_robot_velocity.x(),
            tangent_vector.y() + desired_robot_velocity.y()
        );

        return new PolarVector(result_vector.angle(), result_vector.length() / 2.0);
    }

    public void run( PolarVector desired_movement, double velocity_multiplier){
        if(desired_movement.length == 0.0){
            move_and_pivot(new PolarVector(last_rotation, 0.0));
            return;
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
        Rotation2d perpendicular_angle = current_angle.plus(Rotation2d.fromDegrees(90.0));
        Rotation2d opposite_angle = current_angle.plus(Rotation2d.fromDegrees(180.0));
    
        // establishes the three angles used to determine the optimal way to move the wheel
        Rotation2d delta = angle_between( current_angle , desired_state.angle );
        Rotation2d zeta = angle_between( perpendicular_angle , desired_state.angle );
        Rotation2d gamma = angle_between( opposite_angle , desired_state.angle );
        if (gamma.getDegrees() < delta.getDegrees()){ // checks for the smallest distance to the target angle

            double power = get_PID( gamma ); // minimize gamma
            
            if (zeta.getDegrees() > 90.0){ // Checks if left or right movement is needed
                move_turn_motor( power );
            }else{
                move_turn_motor( -power );
            }
            
            move_go_motor( -desired_state.length ); // forwards is backwards
        }else{
            
            double power = get_PID( delta ); // minimize delta
    
            if (zeta.getDegrees() > 90.0){ // Checks if left or right movement is needed
                move_turn_motor( -power );
            }else{
                move_turn_motor( power );
            }
    
            move_go_motor( desired_state.length ); // forwards is forwards
        }
    }
    
    // uses dot product formula to find the angle between the two angles
    public Rotation2d angle_between( Rotation2d angle_1 , Rotation2d angle_2 ){
        return Rotation2d.fromRadians(
            Math.acos(
                angle_1.getCos() * angle_2.getCos() + 
                angle_1.getSin() * angle_2.getSin()
            )
        );
    }

    public void move_turn_motor( double power ){
        if (power > 1.0){
            turn_wheel.set( 1.0 * Constants.rotational_dampen_multiplier );
            return;
        }

        if (power < -1.0){
            turn_wheel.set( -1.0 * Constants.rotational_dampen_multiplier );
            return;
        }

        turn_wheel.set( power * Constants.rotational_dampen_multiplier );
    }

    public void move_go_motor( double power ){
        if (power > 1.0){
            move_wheel.set( 1.0 * Constants.rotational_dampen_multiplier );
            return;
        }

        if (power < -1.0){
            move_wheel.set( -1.0 * Constants.rotational_dampen_multiplier );
            return;
        }
        move_wheel.set( power * Constants.speed_dampen_multiplier); 
    }
}