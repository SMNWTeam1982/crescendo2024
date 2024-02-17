package frc.robot.Subsystems.NewSwerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Utilities.PID;

public class SwerveModule {
    public final CANSparkMax go_motor;
    public final RelativeEncoder go_motor_encoder;
    public final CANSparkMax turn_motor;
    public final CANcoder encoder;
    public final Rotation2d encoder_offset;
    private final PID pid = new PID(
        Constants.DriveConstants.wheel_rotation_p,
        Constants.DriveConstants.wheel_rotation_i,
        Constants.DriveConstants.wheel_rotation_d
    );

    public SwerveModule(int go_motor_channel, int turn_motor_channel, int encoder_channel, Rotation2d encoder_offset){
        go_motor = new CANSparkMax(go_motor_channel, MotorType.kBrushless);
        go_motor_encoder = go_motor.getEncoder();
        turn_motor = new CANSparkMax(turn_motor_channel, MotorType.kBrushless);
        encoder = new CANcoder(encoder_channel);
        this.encoder_offset = encoder_offset;

        zero_wheel_position();

        go_motor.restoreFactoryDefaults();
        go_motor.setClosedLoopRampRate(1.0);
        go_motor.setOpenLoopRampRate(0.5);
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
        return go_motor_encoder.getPosition() * Constants.DriveConstants.wheel_rotations_to_meters_traveled;
    }
    public void zero_wheel_position(){
        go_motor_encoder.setPosition(0.0);
    }

    public Rotation2d get_encoder(){
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition().getValueAsDouble() * 360.0).minus(encoder_offset);
    }

    public double get_PID(Rotation2d angle){
        return pid.out(angle.getDegrees(), 0,0);
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

    public void set_desired_state(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, get_state().angle);
        
        move_go_motor(state.speedMetersPerSecond / DriveConstants.wheel_max_speed_meters_per_second);
        move_turn_motor(pid.out(get_encoder().getDegrees(), state.angle.getDegrees(), 0.0));
        SmartDashboard.putString("Swerve[" + encoder.getDeviceID() + "] state", state.toString());
    }

    public void move_turn_motor( double power ){
        if (power > 1.0){
            turn_motor.set( 1.0 * Constants.DriveConstants.wheel_rotation_multiplier );
            return;
        }

        if (power < -1.0){
            turn_motor.set( -1.0 * Constants.DriveConstants.wheel_rotation_multiplier );
            return;
        }

        turn_motor.set( power * Constants.DriveConstants.wheel_rotation_multiplier );
    }

    public void move_go_motor( double power ){
        if (power > 1.0){
            go_motor.set( 1.0 * Constants.DriveConstants.wheel_speed_multiplier );
            return;
        }

        if (power < -1.0){
            go_motor.set( -1.0 * Constants.DriveConstants.wheel_speed_multiplier );
            return;
        }
        go_motor.set( power * Constants.DriveConstants.wheel_speed_multiplier); 
    }

    public void stop() {
        move_go_motor(0);
        move_turn_motor(0);
    }
}
