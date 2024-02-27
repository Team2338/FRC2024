package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Collector extends SubsystemBase {

    public static TalonSRX collector;
    public static DigitalInput sensor;

    public boolean collectorManualControl = false;

    public Collector(){
        collector = new TalonSRX(RobotMap.COLLECTOR_ID);
        collector.configFactoryDefault();
        collector.setNeutralMode(NeutralMode.Brake);

        sensor = new DigitalInput(RobotMap.SENSOR_COLLECTOR_PORT);
    }

    public void collect() {
        collector.set(ControlMode.PercentOutput, Constants.Collector.COLLECT_PERCENT);
    }

    public void eject() {
        collector.set(ControlMode.PercentOutput, -Constants.Collector.EJECT_PERCENT);
    }

    public void stop() {
        collector.set(ControlMode.PercentOutput, 0);
    }

    public boolean getSensorState() {
        return sensor.get();
    }

    public boolean getCollectorManualControl() {
        return collectorManualControl;
    }

    public double getMotorTemp() {
        return collector.getTemperature();
    }

    public boolean isStageOneMotorCool() {
        if (getMotorTemp() >= Constants.MotorTemps.COLLECTOR_MOTOR_TEMP) {
            return false;
        }
        return true;
    }
}
