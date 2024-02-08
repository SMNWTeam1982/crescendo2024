package frc.robot.Utilities;

import edu.wpi.first.math.geometry.Rotation2d;

public class PolarVector {
    public Rotation2d angle; // degrees
    public double length;

    public PolarVector(Rotation2d angle, double length){
        this.angle = angle;
        this.length = length;
    }

    public double x(){
        return angle.getCos() * length;
    }

    public double y(){
        return angle.getSin() * length;
    }
}
