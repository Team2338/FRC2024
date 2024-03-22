// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team.gif.lib.autoMode;
import team.gif.robot.commands.drivetrain.AutoRotate;
import team.gif.robot.commands.autos.NoAuto;
import team.gif.robot.commands.shooter.RevFlyWheels;
import team.gif.robot.commands.shooter.Shoot;

import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private double autonomousInitialHeading;

    private final HashMap<autoMode, Command> autoCommands = new HashMap<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Used in "Paths"
        NamedCommands.registerCommand("AutonRevFlywheel", new RevFlyWheels(true)); // will move wrist to next target position
        NamedCommands.registerCommand("AutonWristFar3", new InstantCommand(Robot.wrist::setWristFar3Position));
        NamedCommands.registerCommand("AutonWristFar2", new InstantCommand(Robot.wrist::setWristFar2Position));
        NamedCommands.registerCommand("AutonWristFar", new InstantCommand(Robot.wrist::setWristFarPosition));
        NamedCommands.registerCommand("AutonWristMidMidFar", new InstantCommand(Robot.wrist::setWristMidMidFarPosition));
        NamedCommands.registerCommand("AutonWristMidFar", new InstantCommand(Robot.wrist::setWristMidFarPosition));
        NamedCommands.registerCommand("AutonWristMid", new InstantCommand(Robot.wrist::setWristMidPosition));
        NamedCommands.registerCommand("AutonWristMiddle", new InstantCommand(Robot.wrist::setWristMiddlePosition));
        NamedCommands.registerCommand("AutonWristNear", new InstantCommand(Robot.wrist::setWristNearPosition));
        NamedCommands.registerCommand("AutonWristClose", new InstantCommand(Robot.wrist::setWristClosePosition));
        NamedCommands.registerCommand("AutonWristWall", new InstantCommand(Robot.wrist::setWristWallPosition));

        // Used in "Autos" - shoots (and may set wrist first0
        NamedCommands.registerCommand("AutonShoot", new Shoot(true));
        NamedCommands.registerCommand("AutonShootWall", new InstantCommand(Robot.wrist::setWristWallPosition).andThen(new Shoot(true))); // used for first shot
        NamedCommands.registerCommand("AutonRotate", new AutoRotate());

        // Configure the trigger bindings
        configureBindings();
        buildAutoCommands();

        autonomousInitialHeading = 0;
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
    }

    private void buildAutoCommands(){
        autoCommands.put(autoMode.NONE, new NoAuto());
        autoCommands.put(autoMode.CIRCLE, AutoBuilder.followPath(PathPlannerPath.fromPathFile("Circle Path")));
//        autoCommands.put(autoMode.MOBILITY, AutoBuilder.followPath(PathPlannerPath.fromPathFile("Mobility")));
        autoCommands.put(autoMode.MOBILITY, new PathPlannerAuto("Mobility"));
        autoCommands.put(autoMode.TWO_CTR_C, new PathPlannerAuto("2CTR+C"));
        autoCommands.put(autoMode.TWO_SRC_S, new PathPlannerAuto("2SRC+S"));
        autoCommands.put(autoMode.TWO_SRC_8, new PathPlannerAuto("2SRC+8"));
        autoCommands.put(autoMode.TWO_SRC_7, new PathPlannerAuto("2SRC+7"));
        autoCommands.put(autoMode.THREE_AMP_A_4_5, new PathPlannerAuto("3AMP+A-4+5"));
        autoCommands.put(autoMode.THREE_W_8_7, new PathPlannerAuto("3W+8+7"));
        autoCommands.put(autoMode.FOUR_AMP_A_C_S, new PathPlannerAuto("4AMP+A+C+S"));
        autoCommands.put(autoMode.FIVE_SRC_S_C_A_4, new PathPlannerAuto("5SRC+S+C+A+4"));
        autoCommands.put(autoMode.FIVE_CTR_C_S_A_4, new PathPlannerAuto("5CTR+C+S+A+4"));
        autoCommands.put(autoMode.FIVE_CTR_C_S_A_5, new PathPlannerAuto("5CTR+C+S+A+5"));
        autoCommands.put(autoMode.TWO_SCSPLIT_SIX, new PathPlannerAuto("2SCSplit+6"));
        autoCommands.put(autoMode.LINE_TEST, new PathPlannerAuto("Straight Line Test"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(autoMode chosenAuto) {
        Pose2d startingPose;
        Command autonomousCommand = autoCommands.get(chosenAuto);

        if (chosenAuto == null) {
            System.out.println("Autonomous selection is null. Robot will do nothing in auto :(");
        } else {
            startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autonomousCommand.getName());
            autonomousInitialHeading = startingPose.getRotation().getDegrees();
        }

        return autonomousCommand;
    }

    public double getAutonomousInitialHeading(){
        return  autonomousInitialHeading;
    }
}
