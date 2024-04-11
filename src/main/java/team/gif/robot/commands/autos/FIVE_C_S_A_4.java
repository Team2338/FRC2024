package team.gif.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team.gif.robot.Robot;
import team.gif.robot.commands.autos.lib.AutonNoNote;
import team.gif.robot.commands.drivetrain.AutoRotate;
import team.gif.robot.commands.shooter.RevFlyWheels;
import team.gif.robot.commands.shooter.Shoot;

public class FIVE_C_S_A_4 extends SequentialCommandGroup {
    public FIVE_C_S_A_4() {
        addCommands(
                new InstantCommand(Robot.wrist::setNextShotWall).andThen(new Shoot(true)), //new Shoot(true),
                new ParallelRaceGroup(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("5CTR+C-Away")),
                        new RevFlyWheels(true)
                ),
                new Shoot(true),
                new ParallelRaceGroup(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("5CTR+C+S")),
                        new RevFlyWheels(true)
                ),
                new Shoot(true),
                new ParallelRaceGroup(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("5CTR+C+S+A")),
                        new RevFlyWheels(true)
                ),
                new Shoot(true),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("5CTR+C+S+A+4")),

                // Decision tree
                Commands.either(
                        // if the robot has a note, run the following commands
                        new SequentialCommandGroup(
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("5CTR+C+S+A+4-Return")),
                                new AutoRotate(),
                                new Shoot(true)
                        ),
                        // if the robot does not have a note, run the following commands
                        new SequentialCommandGroup(
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("5CTR+C+S+A+4-Abort+5")),
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile("5CTR+C+S+A+5-Return")),
                                new AutoRotate(),
                                new Shoot(true)
                        ),
                        // check if the robot has a note
                        Robot.diagnostics.getRobotHasNoteSupplier()
                )
        );
    }
}
