package team.gif.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Flapper extends SubsystemBase {
    public static Servo servo;

    public Flapper(int port){
        servo = new Servo(port);
    }

    public void setHorizontal() {
        servo.set(0.65);
    }

    public void setVertical() {
        servo.set(0.0);
    }
}
