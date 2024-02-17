package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.NewSwerve.SwerveSubsystem;

public class FieldOrient extends Command{

    private final SwerveSubsystem drive;

    private double start_time = Timer.getFPGATimestamp();

    public FieldOrient(SwerveSubsystem drive) {
        this.drive = drive;
    
        addRequirements(drive);
        // Use addRequirements() here to declare subsystem dependencies.
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!Limelight.visible_speaker()){
            return;
        }

        Rotation2d robot_rotation = Limelight.get_field_rotation();

        drive.set_gyro_offset(robot_rotation);

        end(false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - start_time >= 1.0); // gives up after 1 second
    }
}
