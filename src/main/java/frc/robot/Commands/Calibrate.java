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
    private final Shooter shooter;
    private final Intake intake;

    private boolean drive_calibrated = false;
    private boolean shooter_calibrated = false;
    private boolean intake_calibrated = false;

    private final Command calibrate_intake_command;
    private final Command calibrate_shooter_command;

    private double start_time = Timer.getFPGATimestamp();

    public Calibrate(Drive drive, Intake intake, Shooter shooter) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;

        addRequirements(drive, intake, shooter);

        calibrate_intake_command = intake.get_calibrate_command();
        calibrate_shooter_command = shooter.get_calibrate_command();

        CommandScheduler.getInstance().schedule(calibrate_intake_command,calibrate_shooter_command);
    }
    
    @Override
    public void initialize() {
        
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(drive.attempt_set_team() && drive.attempt_field_orient() && !drive_calibrated){
            drive_calibrated = true;
        }

        intake_calibrated = !calibrate_intake_command.isScheduled();
        shooter_calibrated = !calibrate_shooter_command.isScheduled();

        if(drive_calibrated && intake_calibrated && shooter_calibrated){
            end(false);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().cancel(calibrate_intake_command,calibrate_shooter_command);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - start_time >= 1.5); // gives up after 1.5 seconds
    }
}
