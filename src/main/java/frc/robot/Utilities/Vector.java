package frc.robot.Utilities;

import edu.wpi.first.math.geometry.Rotation2d;

public class Vector {
    public double y;
    public double x;

    public Vector(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Rotation2d angle(){
        return Rotation2d.fromRadians(Math.atan2(y,x));
    }

    public double length(){
        return Math.sqrt(x*x + y*y);
    }
}
