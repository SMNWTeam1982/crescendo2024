// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.Drive;
import frc.robot.Utilities.PolarVector;
import frc.robot.Utilities.Rotation2dFix;
import frc.robot.Utilities.Vector;
import frc.robot.Commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Auto;
import frc.robot.Subsystems.Climbers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.HashMap;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }
  
  public final Drive swerve_drive = new Drive();
  private final Shooter shooter = new Shooter(13, 11, 12);
  private final Intake intake = new Intake(9, 10, 9);
  private final Climbers climbers = new Climbers();
  private final XboxController drive_controller = new XboxController(0);
  private final XboxController shooter_controller = new XboxController(1);
  //private final Button aim_button = new
  
  private static final HashMap<Command,Command> sequenceMap = new HashMap<Command,Command>();
  public static SendableChooser<Command> m_chooser=new SendableChooser<>();

  public void configureBindings() {
    swerve_drive.BuilderConfigure();

    swerve_drive.setDefaultCommand(
            new Swerve(
              swerve_drive,
              () -> {
                Vector inputs = new Vector(-drive_controller.getLeftX(), drive_controller.getLeftY());

                double speed = Math.sqrt(inputs.x * inputs.x + inputs.y * inputs.y);
                if(speed > 1.0){speed = 1.0;}
                if(speed < 0.2){speed = 0.0;}
          
                PolarVector desired_velocity = new PolarVector(
                  Rotation2d.fromRadians(Math.atan2(inputs.y,inputs.x)).plus(Rotation2d.fromDegrees(90.0)),
                  speed
                );

                return desired_velocity;
              },
              () -> {
                if(drive_controller.getAButton()){
                  Pose2d robot_position = swerve_drive.get_robot_pose();
                  Rotation2d desired_angle = new Rotation2d();
                  if(swerve_drive.is_on_red_side()){
                    desired_angle = Rotation2dFix.fix(Constants.ShooterConstants.red_speaker_position.minus(robot_position.getTranslation()).getAngle());
                  }else{
                    desired_angle = Rotation2dFix.fix(Constants.ShooterConstants.blue_speaker_position.minus(robot_position.getTranslation()).getAngle());
                  }
                  return swerve_drive.get_rotation_pid(desired_angle);
                }

                double x = drive_controller.getRightX();
                if(Math.abs(x) < 0.1){x = 0.0;}
                return x;
              },
              () -> {return (drive_controller.getRightTriggerAxis() < 0.1);},
              // () -> {return drive_controller.getAButton();},
              () -> {return drive_controller.getYButtonReleased();}
            )
          );
          shooter.setDefaultCommand(
            shooter.shooter_command(
              () -> {return (shooter_controller.getRightTriggerAxis() > 0.1);},
              () -> {
                if( shooter_controller.getLeftTriggerAxis() > 0.1){
                  return Constants.ShooterConstants.shooter_climb_angle;
                }
                if( shooter_controller.getRightTriggerAxis() < 0.1){
                  return Constants.ShooterConstants.shooter_start_angle.minus( Rotation2d.fromDegrees(5.0) );
                }

                Pose2d robot_position = swerve_drive.get_robot_pose();

                Translation2d shooter_pivot_position = robot_position.getTranslation().plus(
                  new Translation2d(Constants.ShooterConstants.shooter_pivot_point_forward,robot_position.getRotation())
                );

                double dist_meters = 0.0;
                if(swerve_drive.is_on_red_side()){
                  dist_meters = shooter_pivot_position.getDistance(Constants.ShooterConstants.red_speaker_position);
                }else{
                  dist_meters = shooter_pivot_position.getDistance(Constants.ShooterConstants.blue_speaker_position);
                }
                SmartDashboard.putNumber("dist",dist_meters);
                return Rotation2d.fromRadians(Math.atan(Constants.ShooterConstants.speaker_height_difference / dist_meters));
                
                //return Constants.ShooterConstants.shooter_load_angle; // math not implemented yet
              }
            )
          );

          intake.setDefaultCommand(
            intake.get_intake_command(
              () -> {
                return shooter_controller.getLeftY();
              },
              () -> {
                Rotation2d angle = Rotation2d.fromDegrees(0.0);
                if(shooter_controller.getYButton()){
                  return angle.plus(Constants.IntakeConstants.amp_scoring_angle);
                }
                if(shooter_controller.getBButton()){
                  return angle.plus(Constants.IntakeConstants.deployed_angle);
                }
                if(shooter_controller.getXButton()){
                  return angle.plus(Constants.IntakeConstants.handoff_angle);
                }
                return null;
              }
            )
          );

          climbers.setDefaultCommand(
              climbers.get_climb_command(() -> {
                return shooter_controller.getPOV();
              }
            )
          );

    //GenerateCommands();

  }

  public void GenerateCommands(){
        m_chooser = new SendableChooser<>();
        
        //Command leavecommunity = stitchSequential(Commands.waitSeconds(1.7) , new AutoArmPneumatics(pneumatics) , Commands.waitSeconds(.5) ,  new AutoMove(drive, -168));
        //m_chooser.addOption("Move Out Of Community", leavecommunity);
        
      //  Command test = stitchSequential(new PathPlannerAuto("test"));
    //    m_chooser.addOption("test", test);

        SmartDashboard.putData("Auto chooser", m_chooser);
        SmartDashboard.updateValues();
    }
    
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

  public Command getAutonomousCommand() {
    //return AutoBuilder.followPath(PathPlannerPath.fromPathFile("test"));
    return stitchSequential(
      Commands.runOnce(
        () -> {
          intake.setDefaultCommand(intake.idle_command());
          shooter.setDefaultCommand(shooter.idle_command());
          swerve_drive.setDefaultCommand(swerve_drive.idle_command());
        }
      ),
      new Calibrate(swerve_drive),
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
      shooter.set_angle_command(Rotation2d.fromDegrees(30.0)),
      new WaitCommand(3.0),
      intake.eject_command(),
      Commands.runOnce(
        () -> {
          swerve_drive.setDefaultCommand(
            new Swerve(
              swerve_drive,
              () -> {
                Vector inputs = new Vector(-drive_controller.getLeftX(), drive_controller.getLeftY());

                double speed = Math.sqrt(inputs.x * inputs.x + inputs.y * inputs.y);
                if(speed > 1.0){speed = 1.0;}
                if(speed < 0.1){speed = 0.0;}
          
                PolarVector desired_velocity = new PolarVector(
                  Rotation2d.fromRadians(Math.atan2(inputs.y,inputs.x)).plus(Rotation2d.fromDegrees(90.0)),
                  speed
                );

                return desired_velocity;
              },
              () -> {
                if(drive_controller.getAButton()){
                  Pose2d robot_position = swerve_drive.get_robot_pose();
                  Rotation2d desired_angle = new Rotation2d();
                  if(swerve_drive.is_on_red_side()){
                    desired_angle = Rotation2dFix.fix(Constants.ShooterConstants.red_speaker_position.minus(robot_position.getTranslation()).getAngle());
                  }else{
                    desired_angle = Rotation2dFix.fix(Constants.ShooterConstants.blue_speaker_position.minus(robot_position.getTranslation()).getAngle());
                  }
                  return swerve_drive.get_rotation_pid(desired_angle);
                }

                double x = drive_controller.getRightX();
                if(Math.abs(x) < 0.1){x = 0.0;}
                return x;
              },
              () -> {return (drive_controller.getRightTriggerAxis() < 0.1);},
              // () -> {return drive_controller.getAButton();},
              () -> {return drive_controller.getYButtonReleased();}
            )
          );
          shooter.setDefaultCommand(
            shooter.shooter_command(
              () -> {return (shooter_controller.getRightTriggerAxis() > 0.1);},
              () -> {
                if( shooter_controller.getLeftTriggerAxis() > 0.1){
                  return Constants.ShooterConstants.shooter_climb_angle;
                }
                if( shooter_controller.getRightTriggerAxis() < 0.1){
                  return Constants.ShooterConstants.shooter_start_angle.minus( Rotation2d.fromDegrees(5.0) );
                }

                Pose2d robot_position = swerve_drive.get_robot_pose();

                Translation2d shooter_pivot_position = robot_position.getTranslation().plus(
                  new Translation2d(Constants.ShooterConstants.shooter_pivot_point_forward,robot_position.getRotation())
                );

                double dist_meters = 0.0;
                if(swerve_drive.is_on_red_side()){
                  dist_meters = shooter_pivot_position.getDistance(Constants.ShooterConstants.red_speaker_position);
                }else{
                  dist_meters = shooter_pivot_position.getDistance(Constants.ShooterConstants.blue_speaker_position);
                }
                SmartDashboard.putNumber("dist",dist_meters);
                return Rotation2d.fromRadians(Math.atan(Constants.ShooterConstants.speaker_height_difference / dist_meters));
                
                //return Constants.ShooterConstants.shooter_load_angle; // math not implemented yet
              }
            )
          );

          intake.setDefaultCommand(
            intake.get_intake_command(
              () -> {
                return shooter_controller.getLeftY();
              },
              () -> {
                if(shooter_controller.getYButton()){
                  return 75.0;
                }
                if(shooter_controller.getBButton()){
                  return 195.0;
                }
                if(shooter_controller.getXButton()){
                  return 10.0;
                }
                return -1;
              }
            )
          );

          climbers.setDefaultCommand(
              climbers.get_climb_command(() -> {
                return shooter_controller.getPOV();
              }
            )
          );
        }
      )
    );
      // new ParallelCommandGroup(
      //   new Swerve(
      //     swerve_drive,
      //     () -> new PolarVector(Rotation2d.fromDegrees(0.0),1.0),
      //     () -> 0.0,
      //     () -> true,
      //     () -> false
      //   ),
      //   new WaitCommand(0.1)
      // )
  }
}
