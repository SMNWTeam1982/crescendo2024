package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase{
    private TalonSRX left_motor = new TalonSRX(20);
    private TalonSRX right_motor = new TalonSRX(21);

    


    public Command get_climb_command(
        Supplier<Double> left_down,
        Supplier<Double> right_down,
        Supplier<Boolean> left_up,
        Supplier<Boolean> right_up
    ){

        return run(
            () -> {
                double left_down_input = left_down.get();
                double right_down_input = right_down.get();

                double left_power = 0.0;
                if(left_up.get()){
                    left_power = 1.0;
                }else if(left_down_input > 0.05){
                    left_power = -left_down_input;
                }
                double right_power = 0.0;
                if(right_up.get()){
                    right_power = 1.0;
                }else if(right_down_input > 0.05){
                    right_power = -right_down_input;
                }

                set_climbers(left_power, right_power);
            }
        );
    }

    private void set_climbers(double left,double right){
        left_motor.set(ControlMode.PercentOutput, left * Constants.ClimberConstants.left_multiplier);
        right_motor.set(ControlMode.PercentOutput, right * Constants.ClimberConstants.right_multiplier);
    }

}
