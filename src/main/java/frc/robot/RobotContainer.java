// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.HashMap;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }
  
  private final Drive swerve_drive = new Drive();
  private final Shooter shooter = new Shooter(13, 11, 12);
  private final Intake intake = new Intake(9, 10, 0);
  private final XboxController drive_controller = new XboxController(0);
  private final XboxController shooter_controller = new XboxController(1);
  
  private static final HashMap<Command,Command> sequenceMap = new HashMap<Command,Command>();
  public static SendableChooser<Command> m_chooser=new SendableChooser<>();

  private void configureBindings() {
    //GenerateCommands();
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
              desired_angle = Rotation2dFix.fix(Constants.ShooterConstants.red_speaker_position.minus(robot_position.getTranslation()).getAngle().minus(Rotation2d.fromDegrees(2.0)));
            }else{
              desired_angle = Rotation2dFix.fix(Constants.ShooterConstants.blue_speaker_position.minus(robot_position.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(2.0)));
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
            return Constants.IntakeConstants.amp_scoring_angle;
          }
          if(shooter_controller.getBButton()){
            return Constants.IntakeConstants.deployed_angle;
          }
          if(shooter_controller.getXButton()){
            return Constants.IntakeConstants.handoff_angle;
          }
          return null;
        }
      )
    );
  }

  // public void GenerateCommands(){
  //       m_chooser = new SendableChooser<>();
        
  //       //Command leavecommunity = stitchSequential(Commands.waitSeconds(1.7) , new AutoArmPneumatics(pneumatics) , Commands.waitSeconds(.5) ,  new AutoMove(drive, -168));
  //       //m_chooser.addOption("Move Out Of Community", leavecommunity);
        
  //       Command test = stitchSequential(new PathPlannerAuto("test"));
  //       m_chooser.addOption("test", test);

  //       SmartDashboard.putData("Auto chooser", m_chooser);
  //       SmartDashboard.updateValues();
  //   }
    
  //   public static Command stitchSequential(Command... commands){
  //       for(int i=0;i<commands.length-1;i++){
  //           if(i<commands.length+1)
  //               sequenceMap.put(commands[i],commands[i+1]);
  //       }
 
        
  //       CommandScheduler.getInstance().onCommandFinish(
  //           (command)->{
  //               if(sequenceMap.containsKey(command))
  //               CommandScheduler.getInstance().schedule(sequenceMap.get(command));
  //           }
  //       );
        
  //       return commands[0];
  //   }

  public Command getAutonomousCommand() {
    return null;
  }
}
