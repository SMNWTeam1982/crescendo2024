// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Limelight;
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
  private final CommandXboxController command_shooter_controller = new CommandXboxController(1);
  //private final Button aim_button = new
  
  private static final HashMap<Command,Command> sequenceMap = new HashMap<Command,Command>();

  public static SendableChooser<Command> chooser = new SendableChooser<Command>();

  public void configureBindings() {
    swerve_drive.BuilderConfigure();

    command_shooter_controller.rightBumper().onTrue(intake.grab_note_command());
    command_shooter_controller.x().onTrue(intake.handoff_command());
    //command_shooter_controller.y().onTrue(intake.set_angle_command(Constants.IntakeConstants.amp_scoring_angle));
    command_shooter_controller.b().onTrue(intake.deploy_command());

    command_shooter_controller.axisLessThan(1, -0.5)
      .onTrue(intake.eject_command())
      .onFalse(intake.stop_command());
    command_shooter_controller.axisGreaterThan(1, 0.5)
      .onTrue(intake.suck_command())
      .onFalse(intake.stop_command());

    // command_shooter_controller.axisGreaterThan(3/*right trigger*/,0.5)
    //   .onTrue(shooter.start_motors_command())
    //   .onFalse(shooter.stop_motors_command());
    // command_shooter_controller.leftBumper()
    //   .onTrue(shooter.s)

    set_teleOp_commands();
    //GenerateCommands();
    // chooser.setDefaultOption("2-note", two_note_auto(swerve_drive, shooter, intake));
    // chooser.addOption("left auto", Auto.get_auto_left(swerve_drive, shooter, intake));
    // chooser.addOption("right auto", Auto.get_auto_right(swerve_drive, shooter, intake));
    // SmartDashboard.putData("Auto chooser", chooser);
    // SmartDashboard.updateValues();
  }

  public void GenerateCommands(){
        
      //  Command leavecommunity = stitchSequential(Commands.waitSeconds(1.7) , new AutoArmPneumatics(pneumatics) , Commands.waitSeconds(.5) ,  new AutoMove(drive, -168));
      //  chooser.addOption("Move Out Of Community", leavecommunity);
        
      Command test = stitchSequential(new PathPlannerAuto("test"));
      chooser.addOption("test", test);
        

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

  
    // public static Command two_note_auto(Drive swerve_drive, Shooter shooter, Intake intake){
    //   // return new SequentialCommandGroup(null)
    //   return new SequentialCommandGroup(
    //     new Calibrate(swerve_drive),
    //     new WaitCommand(1.0),
    //     //swerve_drive.auto_aim_command(),
    //     shooter.start_motors_command(),
    //     shooter.set_angle_command(Rotation2d.fromDegrees(50.0)),
    //     new WaitCommand(1.0),
    //     intake.eject_command(),
    //     new WaitCommand(1.0),
    //     shooter.stop_motors_command(),
    //     intake.suck_command(),
    //     intake.set_angle_command(Constants.IntakeConstants.deployed_angle),
    //     swerve_drive.go_in_direction_command(new PolarVector(Rotation2d.fromDegrees(0.0), 0.2)),
    //     new WaitCommand(2.5),
    //     swerve_drive.stop_command(),
    //     intake.set_angle_command(Constants.IntakeConstants.handoff_angle),
    //     shooter.start_motors_command(),
    //     shooter.set_angle_command(Rotation2d.fromDegrees(33.0)),
    //     new WaitCommand(3.0),
    //     intake.eject_command()
    //   );
    // }
  

  public Command getAutonomousCommand() {
    // return stitchSequential(
    //   Commands.runOnce(
    //     () -> {
    //       intake.setDefaultCommand(intake.idle_command());
    //       shooter.setDefaultCommand(shooter.idle_command());
    //       swerve_drive.setDefaultCommand(swerve_drive.idle_command());
    //     }
    //   ),
    //   chooser.getSelected(),
    //   Commands.runOnce( () -> set_teleOp_commands() )
    // );
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile("test"));
    // return stitchSequential(
    //   Commands.runOnce(
    //     () -> {
    //       intake.setDefaultCommand(intake.idle_command());
    //       shooter.setDefaultCommand(shooter.idle_command());
    //       swerve_drive.setDefaultCommand(swerve_drive.idle_command());
    //     }
    //   ),
    //   new Calibrate(swerve_drive),
    //   new WaitCommand(1.0),
    //   //swerve_drive.auto_aim_command(),
    //   shooter.start_motors_command(),
    //   shooter.set_angle_command(Rotation2d.fromDegrees(50.0)),
    //   new WaitCommand(1.0),
    //   intake.eject_command(),
    //   new WaitCommand(1.0),
    //   shooter.stop_motors_command(),
    //   intake.set_angle_command(Constants.IntakeConstants.deployed_angle),
    //   intake.suck_command(),
    //   swerve_drive.go_in_direction_command(new PolarVector(Rotation2d.fromDegrees(0.0), 0.2)),
    //   new WaitCommand(2.5),
    //   swerve_drive.stop_command(),
    //   intake.set_angle_command(Constants.IntakeConstants.handoff_angle),
    //   shooter.start_motors_command(),
    //   shooter.set_angle_command(Rotation2d.fromDegrees(33.0)),
    //   new WaitCommand(3.0),
    //   intake.eject_command(),
    //   Commands.runOnce(
    //     () -> {
    //       set_teleOp_commands();
    //     }
    //   )
    // );
  }

  public void set_teleOp_commands(){
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
                if(drive_controller.getAButton() && Limelight.visible_speaker()){
                  return swerve_drive.get_rotation_pid();
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

                return 0.75;
                // Translation2d robot_position = swerve_drive.get_robot_pose().getTranslation();
                // double dist_meters = 0.0;
                // if(swerve_drive.is_on_red_side()){
                //   dist_meters = robot_position.getDistance(Constants.ShooterConstants.red_speaker_position);
                // }else{
                //   dist_meters = robot_position.getDistance(Constants.ShooterConstants.blue_speaker_position);
                // }

                

                // return MathUtil.clamp(
                //   0.5 + (dist_meters / 12.0),
                //   0.0,
                //   1.0
                // );
              },
              () -> {

                if(shooter_controller.getLeftBumper()){
                  return Constants.ShooterConstants.shooter_override_angle;
                }
                if( shooter_controller.getLeftTriggerAxis() > 0.1){
                  return Constants.ShooterConstants.shooter_climb_angle;
                }

                return Constants.ShooterConstants.shooter_default_angle;

                // if( shooter_controller.getRightTriggerAxis() < 0.1){//default
                //   return Constants.ShooterConstants.shooter_default_angle;
                // }

                // Translation2d robot_position = swerve_drive.get_robot_pose().getTranslation();

                // Translation2d shooter_pivot_position = robot_position.getTranslation().plus(
                //   new Translation2d(Constants.ShooterConstants.shooter_pivot_point_forward,robot_position.getRotation())
                // );

                // double dist_meters = 0.0;
                // if(swerve_drive.is_on_red_side()){
                //   dist_meters = robot_position.getDistance(Constants.ShooterConstants.red_speaker_position);
                // }else{
                //   dist_meters = robot_position.getDistance(Constants.ShooterConstants.blue_speaker_position);
                // }
                // SmartDashboard.putNumber("dist",dist_meters);
                // return Rotation2d.fromRadians(Math.atan(Constants.ShooterConstants.speaker_height_difference / dist_meters));
                
                //return Constants.ShooterConstants.shooter_load_angle; // math not implemented yet
              }
            )
          );

          // intake.setDefaultCommand(
          //   intake.get_intake_command(
          //     () -> {
          //       double speed = shooter_controller.getLeftY();
          //       if (Math.abs(speed) < 0.5){
          //         return 0.0;
          //       }
          //       return speed;
          //     },
          //     () -> {
          //       if(shooter_controller.getYButton()){
          //         return Constants.IntakeConstants.amp_scoring_angle;
          //       }
          //       if(shooter_controller.getBButton()){
          //         return Constants.IntakeConstants.deployed_angle;
          //       }
          //       if(shooter_controller.getXButton()){
          //         return Constants.IntakeConstants.handoff_angle;
          //       }
          //       return -1.0;
          //     },
          //     () -> shooter_controller.getAButtonReleased(),
          //     () -> shooter_controller.getRightY(),
          //     () -> shooter_controller.getStartButton()
          //   )
          // );

          climbers.setDefaultCommand(
            climbers.get_climb_command(
              () -> drive_controller.getLeftTriggerAxis(),
              () -> drive_controller.getRightTriggerAxis(),
              () -> drive_controller.getLeftBumper(),
              () -> drive_controller.getRightBumper()
            )
          );
  }
}
