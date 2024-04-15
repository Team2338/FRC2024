package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flapper extends SubsystemBase {

    public static Servo servo;

    public Flapper(int port) {
        servo = new Servo(port);
    }

    public void setHorizontal() {
        servo.set(0.65);
    }

    public void setVertical() {
        servo.set(0.0);
    }
}

