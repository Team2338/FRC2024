package team.gif.robot.subsystems.drivers;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class Collector extends SubsystemBase {
    public TalonSRX motor_775 = new TalonSRX(RobotMap.Collector_Bags);
    /** Creates a new ExampleSubsystem. */
    public Collector() {

    }

    public void Turn(Double Output){
        motor_775.set(TalonSRXControlMode.PercentOutput,Output);
    }

}
