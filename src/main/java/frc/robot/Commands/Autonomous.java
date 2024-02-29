// import java.io.IOException;
// import java.nio.file.Files;
// import java.nio.file.Path;
// import java.nio.file.Paths;
// import java.util.stream.Stream;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.FollowPathHolonomic;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.DriveConstants;



// public class Autonomous {
//     public static Command followPathCommand(String pathName) {
//      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
//       return new FollowPathHolonomic(
//          path,
//          RobotContainer.swerve_drive::getEstimatedPose, // Robot pose supplier
//          RobotContainer.swerve_driven::pgetChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//         RobotContainer.swerve_drive::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
//          new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
//                  new PIDConstants(ModuleConstants.DRIVING_P, ModuleConstants.DRIVING_I, ModuleConstants.DRIVING_D), // Translation PID constants
//                  new PIDConstants(ModuleConstants.TURNING_P, ModuleConstants.TURNING_I, ModuleConstants.TURNING_D), // Rotation PID constants
//                  DriveConstants.MAX_SPEED_METERS_PER_SECOND, // Max module speed, in m/s
//                  DriveConstants.WHEEL_BASE/2, // Drive base radius in meters. Distance from robot center to furthest module.
//                  new ReplanningConfig() //Should the path be replanned if the error grows too large or if a large error spike happens while following the path?
//          ),
//          () -> {
//              // Boolean supplier that controls when the path will be mirrored for the red alliance
//              // This will flip the path being followed to the red side of the field.
//              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

//              var alliance = DriverStation.getAlliance();
//              if (alliance.isPresent()) {
//                  return alliance.get() == DriverStation.Alliance.Red;
//              }
//              return false;
//          },
//        RobotContainer.swerve_drive // Reference to this subsystem to set requirements
//         );
//     }
// }