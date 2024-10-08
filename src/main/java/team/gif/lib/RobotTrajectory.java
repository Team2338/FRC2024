package team.gif.lib;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

/**
 * Singleton class for creating a trajectory for a swerve bot
 * @author Rohan Cherukuri
 * @since 2/14/23
 */
public class RobotTrajectory {
    private RobotTrajectory() {}

    private static RobotTrajectory instance = null;

    public static RobotTrajectory getInstance() {
        if(instance == null) {
            instance = new RobotTrajectory();
        }
        return instance;
    }

//    public SwerveControllerCommand baseSwerveCommand(PathPlannerPath trajectory) {
//        // rest odometry to the initial position of the path
////        Robot.pigeon.resetPigeonPosition( trajectory.getInitialHolonomicPose().getRotation().getDegrees());
////        Robot.swervetrain.resetOdometry(trajectory.getInitialHolonomicPose());
//
//        return new SwerveControllerCommand(
//                trajectory,
//                Robot.swerveDrivetrain::getPose,
//                Constants.Drivetrain.DRIVE_KINEMATICS,
//                // TODO SwerveAuto can remove and add after PID constants are finalized and autos are running well
//                //SA new PIDController(SmartDashboard.getNumber("kPX", 5.0), 0, 0),
//                //SA new PIDController(SmartDashboard.getNumber("kPY", 5.0), 0, 0),
//                //SA new PIDController(SmartDashboard.getNumber("kPTheta", 3.7), 0, 0),
//                new PIDController(Constants.AutoConstants.PX_CONTROLLER, 0, 0),
//                new PIDController(Constants.AutoConstants.PY_CONTROLLER, 0, 0),
//                new ProfiledPIDController(Constants.AutoConstants.P_THETA_CONTROLLER, 0, 0,new TrapezoidProfile.Constraints(1.6,3.0)),
//                Robot.swerveDrivetrain::setModuleStates
//                //true <-- currently not working
////                Robot.swerveDrivetrain
//        );
//    }
}