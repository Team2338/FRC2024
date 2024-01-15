package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.gif.robot.Robot;

public class Shooter extends CommandBase {
    public Shooter(){
        super();

    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        System.out.println("Neo is running. ");
        Robot.sparky.setSparky(2.5);

    }
    @Override
    public boolean isFinished(){
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        Robot.sparky.setSparky(0);
    }
}
