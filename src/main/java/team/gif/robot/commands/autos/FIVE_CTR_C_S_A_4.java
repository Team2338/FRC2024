package team.gif.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Robot;
import team.gif.robot.commands.AutonRevFlywheel;
import team.gif.robot.commands.drivetrain.AutoRotate;
import team.gif.robot.commands.shooter.Shoot;

public class FIVE_CTR_C_S_A_4 extends SequentialCommandGroup {
    public FIVE_CTR_C_S_A_4() {
        addCommands(
            new Shoot(true),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("5CTR+C-Away")),
            new InstantCommand(Robot.wrist::setWristMidPosition),
            new ParallelCommandGroup(
                    new Shoot(true),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("5CTR+C+S"))
            ),
            new InstantCommand(Robot.wrist::setWristMiddlePosition),
            new ParallelCommandGroup(
                    new Shoot(true),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("5CTR+C+S+A"))
            ),
            new InstantCommand(Robot.wrist::setWristMidFarPosition),
        new Shoot(true),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("5CTR+C+S+A+4")));

        //If the robot collects do this
        addCommands(
            new ParallelCommandGroup(
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("5CTR+C+S+A+4+Return")),
                    new SequentialCommandGroup(
                            new InstantCommand(Robot.wrist::setWristMiddlePosition),
                            new AutonRevFlywheel()
                    )
            ),
            new AutoRotate(),
            new Shoot(true));

        //If the robot doesn't run a different path
    }
}
