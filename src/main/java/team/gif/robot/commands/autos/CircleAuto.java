package team.gif.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.commands.autos.lib.SetInitialHeading;

import java.util.HashMap;

public class CircleAuto extends SequentialCommandGroup {
    public CircleAuto(){

        // Load the path you want to follow using its name in the GUI
        PathPlannerPath trajectory = PathPlannerPath.fromPathFile("Circle Path");
        HashMap<String, Command> eventMap = new HashMap<>();

        addCommands(
                new SetInitialHeading(trajectory)
        );
    }
}
