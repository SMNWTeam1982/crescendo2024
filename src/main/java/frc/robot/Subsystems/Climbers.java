package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase{
    private TalonSRX left_motor = new TalonSRX(20);
    private TalonSRX right_motor = new TalonSRX(21);


    public Command get_climb_command(Supplier<Integer> input){

        return run(
            () -> {
                int val = input.get();
                if(val == 0){
                    set_climbers(1.0, 1.0);
                }

                if(val == 45){
                    set_climbers(0.0, 1.0);
                }

                if(val == 135){
                    set_climbers(0.0, -1.0);
                }

                if(val == 180){
                    set_climbers(-1.0, -1.0);
                }

                if(val == 225){
                    set_climbers(-1.0, 0.0);
                }

                if(val == 315){
                    set_climbers(1.0, 0.0);
                }

                if(val == -1){
                    set_climbers(0.0, 0.0);
                }
            }
        );
    }

    private void set_climbers(double left,double right){
        left_motor.set(ControlMode.PercentOutput, left * Constants.ClimberConstants.left_multiplier);
        right_motor.set(ControlMode.PercentOutput, right * Constants.ClimberConstants.right_multiplier);
    }

}
