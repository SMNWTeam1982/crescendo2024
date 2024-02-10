package frc.robot.Utilities;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

public class Tracking {
    public final Pose2d pose;
    public final double timestamp;
    public final Matrix<N3, N1> standard_deviation;

    public Tracking(Pose2d pose, int visible_targets){
        this.pose = pose;
        timestamp = Timer.getFPGATimestamp();
        this.standard_deviation = VecBuilder.fill(
            1.0/visible_targets,
            1.0/visible_targets,
            1.0/visible_targets
        );
    }
}
