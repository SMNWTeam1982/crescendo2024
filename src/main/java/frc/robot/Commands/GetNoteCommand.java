package frc.robot.Commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Limelight;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.NewSwerve.SwerveSubsystem;
import frc.robot.Utilities.PID;
import frc.robot.Utilities.PolarVector;

public class GetNoteCommand extends Command {
    
    private final SwerveSubsystem drive;
    private final Intake intake;

    private final PID rotate_pid = new PID(Constants.DriveConstants.note_rotation_p, 
    Constants.DriveConstants.note_rotation_i, 
    Constants.DriveConstants.note_rotation_d);

    private final PID movement_pid = new PID(Constants.DriveConstants.note_movement_p, 
    Constants.DriveConstants.note_movement_i, 
    Constants.DriveConstants.note_movement_d);

    PhotonCamera camera = new PhotonCamera("photonvision");

    private final Command move_command;
    private final Command intake_command;

    public GetNoteCommand(SwerveSubsystem drive, Intake intake) {
        this.drive = drive;
        this.intake = intake;
        addRequirements(drive, intake);

        move_command = new Swerve(
            this.drive,
            () -> {
                var result = camera.getLatestResult();
                return new PolarVector(
                    Rotation2d.fromDegrees(0),
                    movement_pid.out(result.getBestTarget().getYaw(), 0.0, 0.0)
                );
            },
            () -> {
                var result = camera.getLatestResult();
                return rotate_pid.out(1/result.getBestTarget().getArea(), 0.0, 0.0);
            }
        );

        intake_command = this.intake.get_intake_command(
            () -> {return 1.0;},
            () -> {return Constants.IntakeConstants.deployed_angle;}
        );
        // Use addRequirements() here to declare subsystem dependencies.
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        var result = camera.getLatestResult();
        if (result.hasTargets() == true){
            CommandScheduler.getInstance().schedule(move_command);
            CommandScheduler.getInstance().schedule(intake_command);
        }
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var result = camera.getLatestResult();
        if (result.hasTargets() == false){
            end(false);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().cancel(move_command);
        CommandScheduler.getInstance().cancel(intake_command);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
       return false;
    }
}
