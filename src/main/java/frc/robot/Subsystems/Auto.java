package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;

public class Auto extends SubsystemBase{
    //dumby classs for commandschedueler

  private boolean intakeTargetingShooter = false;
  private double intakeSpeed = 0.0; 
  private Rotation2d intakeAngle = new Rotation2d(0);

  private boolean getIntakeAngleState(){
    return intakeTargetingShooter;
  }

    private Rotation2d getIntakeAngle(){
        return intakeAngle;
    }

  private double getAutoIntakeSpeed(){
    return intakeSpeed;
  }

  private Command deployIntake(){
    return runOnce( ()->{
      intakeSpeed = Constants.IntakeConstants.deployedSpeed;
      //intakeTargetingShooter = false;
      intakeAngle = Constants.IntakeConstants.deployed_angle;
    });
  }

  private Command holdIntake(){
    return runOnce( ()->{
      intakeSpeed = 0;
     // intakeTargetingShooter = true;
     intakeAngle = Constants.IntakeConstants.handoff_angle;
    });
  }

  private Command shootIntake(){
    return runOnce( ()->{
      intakeSpeed = Constants.IntakeConstants.shootSpeed;
      //intakeTargetingShooter = true;
      intakeAngle = Constants.IntakeConstants.amp_scoring_angle;
    });
  }
}
