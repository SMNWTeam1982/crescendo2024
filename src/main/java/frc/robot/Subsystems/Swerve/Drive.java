package frc.robot.Subsystems.Swerve;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Limelight;
import frc.robot.Utilities.PolarVector;

public class Drive extends SubsystemBase {
    private Gyroscope gyro = new Gyroscope();

    private Wheel front_right_wheel = new Wheel(
        new CANSparkMax(5, MotorType.kBrushless),
        new CANSparkMax(6, MotorType.kBrushless),
        new CANcoder(1),
        Constants.front_right_encoder_offset,
        Constants.front_right_tangent
    );
    
    private Wheel front_left_wheel = new Wheel(
        new CANSparkMax(3, MotorType.kBrushless),
        new CANSparkMax(4, MotorType.kBrushless),
        new CANcoder(2),
        Constants.front_left_encoder_offset,
        Constants.front_left_tangent
    );

    private Wheel back_left_wheel = new Wheel(
        new CANSparkMax(1, MotorType.kBrushless),
        new CANSparkMax(2, MotorType.kBrushless),
        new CANcoder(3),
        Constants.back_left_encoder_offset,
        Constants.back_left_tangent
    );

    private Wheel back_right_wheel = new Wheel(
        new CANSparkMax(7, MotorType.kBrushless),
        new CANSparkMax(8, MotorType.kBrushless),
        new CANcoder(4),
        Constants.back_right_encoder_offset,
        Constants.back_right_tangent
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
     * 
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

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                /* one-time action goes here */
            });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
