package frc.robot.Utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class Rotation2dFix {
    public static Rotation2d fix(Rotation2d input){
        double raw = input.getDegrees();

        return Rotation2d.fromDegrees( MathUtil.inputModulus(raw,0.0,360.0) );
    }
}
