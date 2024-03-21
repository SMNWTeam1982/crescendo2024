package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.Calibrate;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Utilities.PolarVector;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;

public class Auto extends SubsystemBase{

  
  private static final HashMap<Command,Command> sequenceMap = new HashMap<Command,Command>();

  public static Command stitchSequential(Command... commands){
    for(int i=0;i<commands.length-1;i++){
      if(i<commands.length+1)
        sequenceMap.put(commands[i],commands[i+1]);
    }

    
    CommandScheduler.getInstance().onCommandFinish(
      (command)->{
        if(sequenceMap.containsKey(command))
        CommandScheduler.getInstance().schedule(sequenceMap.get(command));
      }
    );
    
    return commands[0];
  }

  private static Command two_note_auto(Drive swerve_drive, Shooter shooter, Intake intake){
    // return new SequentialCommandGroup(null)
    return stitchSequential(
      Commands.runOnce(
        () -> {
          intake.setDefaultCommand(intake.idle_command());
          shooter.setDefaultCommand(shooter.idle_command());
          swerve_drive.setDefaultCommand(swerve_drive.idle_command());
        },
        intake,
        shooter,
        swerve_drive
      ),
      new Calibrate(swerve_drive),
      new WaitCommand(1.0),
      //swerve_drive.auto_aim_command(),
      shooter.start_motors_command(),
      shooter.set_angle_command(Rotation2d.fromDegrees(50.0)),
      new WaitCommand(1.0),
      intake.eject_command(),
      new WaitCommand(1.0),
      shooter.stop_motors_command(),
      intake.suck_command(),
      intake.set_angle_command(Constants.IntakeConstants.deployed_angle),
      swerve_drive.go_in_direction_command(new PolarVector(Rotation2d.fromDegrees(0.0), 0.2)),
      new WaitCommand(2.5),
      swerve_drive.stop_command(),
      intake.set_angle_command(Constants.IntakeConstants.handoff_angle),
      shooter.start_motors_command(),
      shooter.set_angle_command(Rotation2d.fromDegrees(33.0)),
      new WaitCommand(3.0),
      intake.eject_command()
    );
  }

  private static Command get_auto_left(Drive swerve_drive, Shooter shooter, Intake intake){
    return stitchSequential(
  Commands.runOnce(
    () -> {
      intake.setDefaultCommand(intake.idle_command());
      shooter.setDefaultCommand(shooter.idle_command());
      swerve_drive.setDefaultCommand(swerve_drive.idle_command());
    }
  ),
  new Calibrate(swerve_drive),
  new WaitCommand(1.0),
  //swerve_drive.auto_aim_command(),
  shooter.start_motors_command(),
  shooter.set_angle_command(Rotation2d.fromDegrees(50.0)),
  new WaitCommand(1.0),
  intake.eject_command(),
  new WaitCommand(1.0),
  shooter.stop_motors_command(),
  new WaitCommand(2.5),
  swerve_drive.go_in_direction_command(new PolarVector(Rotation2d.fromDegrees(45.0), 0.2)),
  new WaitCommand(4),
  swerve_drive.stop_command()
);
}

public static Command get_auto_right(Drive swerve_drive, Shooter shooter, Intake intake){
    return stitchSequential(
  Commands.runOnce(
    () -> {
      intake.setDefaultCommand(intake.idle_command());
      shooter.setDefaultCommand(shooter.idle_command());
      swerve_drive.setDefaultCommand(swerve_drive.idle_command());
    }
  ),
  new Calibrate(swerve_drive),
  new WaitCommand(1.0),
  //swerve_drive.auto_aim_command(),
  shooter.start_motors_command(),
  shooter.set_angle_command(Rotation2d.fromDegrees(50.0)),
  new WaitCommand(1.0),
  intake.eject_command(),
  new WaitCommand(1.0),
  shooter.stop_motors_command(),
  new WaitCommand(2.5),
  swerve_drive.go_in_direction_command(new PolarVector(Rotation2d.fromDegrees(-45.0), 0.2)),
  new WaitCommand(4),
  swerve_drive.stop_command()
);
}
  // private boolean intakeTargetingShooter = false;
  // private double intakeSpeed = 0.0; 
  // private Rotation2d intakeAngle = new Rotation2d(0);

  // private boolean getIntakeAngleState(){
  //   return intakeTargetingShooter;
  // }

  //   private Rotation2d getIntakeAngle(){
  //       return intakeAngle;
  //   }

  // private double getAutoIntakeSpeed(){
  //   return intakeSpeed;
  // }

  // private Command deployIntake(){
  //   return runOnce( ()->{
  //     intakeSpeed = Constants.IntakeConstants.deployedSpeed;
  //     //intakeTargetingShooter = false;
  //     intakeAngle = Constants.IntakeConstants.deployed_angle;
  //   });
  // }

  // private Command holdIntake(){
  //   return runOnce( ()->{
  //     intakeSpeed = 0;
  //    // intakeTargetingShooter = true;
  //    intakeAngle = Constants.IntakeConstants.handoff_angle;
  //   });
  // }

  // private Command shootIntake(){
  //   return runOnce( ()->{
  //     intakeSpeed = Constants.IntakeConstants.shootSpeed;
  //     //intakeTargetingShooter = true;
  //     intakeAngle = Constants.IntakeConstants.amp_scoring_angle;
  //   });
  // }
}
