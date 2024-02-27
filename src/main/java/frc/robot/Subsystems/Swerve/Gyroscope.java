package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class Gyroscope {
    private Pigeon2 gyro = new Pigeon2(0);

    private Rotation2d offset = Rotation2d.fromDegrees(0.0);
    
    public Gyroscope(){
        zero_angle();
    }

    public Rotation2d get_angle(){
        return gyro.getRotation2d().minus(offset);
    }

    public void set_offset(Rotation2d offset){
        this.offset = offset;
    }

    public void zero_angle(){
        offset = gyro.getRotation2d();
    }

    public void field_orient(Rotation2d vision_input){
        offset = gyro.getRotation2d().minus(vision_input);
    }
}
