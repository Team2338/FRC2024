package team.gif.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class runShooter extends Command {
    public runShooter(){
        super();
    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        Robot.shooter.setShooter(2.5);


    }
    @Override
    public boolean isFinished(){
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        Robot.shooter.setShooter(0.0);

    }
}
