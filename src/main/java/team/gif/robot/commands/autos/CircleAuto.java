package team.gif.robot.commands.autos;

import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.lib.RobotTrajectory;

import java.util.HashMap;

public class CircleAuto extends SequentialCommandGroup {
    public CircleAuto(){

        // Load the path you want to follow using its name in the GUI
        PathPlannerPath trajectory = PathPlannerPath.fromPathFile("Circle Path");
        HashMap<String, Command> eventMap = new HashMap<>();

//        FollowPathWithEvents trajectoryWithEvents = new FollowPathWithEvents(
//                RobotTrajectory.getInstance().baseSwerveCommand(trajectory),
//                trajectory.getEventMarkers(),
//                eventMap
//        );

        addCommands(
        );
    }
}
