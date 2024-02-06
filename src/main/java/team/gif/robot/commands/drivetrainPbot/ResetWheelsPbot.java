package team.gif.robot.commands.drivetrainPbot;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class ResetWheelsPbot extends Command {

    public ResetWheelsPbot() {
        //Cap the maximum rate of change
        //hence the max accel.
        addRequirements(Robot.practiceDrivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("INITIALIZED DRIVE COMMAND");
    }

    @Override
    public void execute() {
        Robot.practiceDrivetrain.resetWheels();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.practiceDrivetrain.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}