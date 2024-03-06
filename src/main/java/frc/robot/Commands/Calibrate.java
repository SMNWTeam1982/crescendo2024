package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.Drive;

public class Calibrate extends Command{

    private final Drive drive;

    private double start_time = Timer.getFPGATimestamp();

    private boolean is_field_oriented = false;
    private boolean is_team_set = false;

    public Calibrate(Drive drive) {
        this.drive = drive;


        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(drive.attempt_set_team()){
            is_team_set = true;
        }

        if(drive.attempt_field_orient()){
            is_field_oriented = true;
        }
        if(is_field_oriented && is_team_set){
            end(false);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - start_time >= 0.2); // gives up after 1.5 seconds
    }
}
