// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team.gif.lib.autoMode;
import team.gif.robot.commands.autos.NoAuto;
import team.gif.robot.commands.shooter.Shoot;

import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final HashMap<autoMode, Command> autoCommands = new HashMap<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("AutonShoot", new InstantCommand(Robot.wrist::setWristWallPosition).andThen(new Shoot(true)));

        // Configure the trigger bindings
        configureBindings();
        buildAutoCommands();
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
        autoCommands.put(autoMode.MOBILITY, AutoBuilder.followPath(PathPlannerPath.fromPathFile("Mobility")));
        autoCommands.put(autoMode.TWO_CTR_C, new PathPlannerAuto("2CTR+C"));
        autoCommands.put(autoMode.TWO_SRC_S, new PathPlannerAuto("2SRC+S"));
        autoCommands.put(autoMode.TWO_SRC_8, new PathPlannerAuto("2SRC+8"));
        autoCommands.put(autoMode.TWO_SRC_7, new PathPlannerAuto("2SRC+7"));
        autoCommands.put(autoMode.THREE_AMP_A_4_5, new PathPlannerAuto("3AMP+A-4+5"));
        autoCommands.put(autoMode.THREE_W_8_7, new PathPlannerAuto("3W+8+7"));
        autoCommands.put(autoMode.FOUR_AMP_A_C_S, new PathPlannerAuto("4AMP+A+C+S"));
        autoCommands.put(autoMode.FIVE_SRC_S_C_A_4, new PathPlannerAuto("5SRC+S+C+A+4"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(autoMode chosenAuto) {
        Command autonomousCommand = autoCommands.get(chosenAuto);

        if (chosenAuto == null) {
            System.out.println("Autonomous selection is null. Robot will do nothing in auto :(");
        }

        return autonomousCommand;
//        PathPlannerPath path= PathPlannerPath.fromPathFile("Circle Path");

 //       return AutoBuilder.followPath(path);
    }
}
