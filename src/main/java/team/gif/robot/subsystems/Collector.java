package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Collector extends SubsystemBase {

    public static TalonSRX collector;
    public static DigitalInput sensor;
    public static boolean collectingState;

    Debouncer collectorDebouncer = new Debouncer(Constants.debounceDefault, Debouncer.DebounceType.kBoth);

    public Collector(){
        collector = new TalonSRX(RobotMap.COLLECTOR_ID);
        collector.configFactoryDefault();
        collector.setNeutralMode(NeutralMode.Brake);

        sensor = new DigitalInput(RobotMap.SENSOR_COLLECTOR_PORT);
        collectingState = false;
    }

    public void collect() {
        collectingState = true;
        collector.set(ControlMode.PercentOutput, Constants.Collector.COLLECT_PERCENT);
    }

    public void reverse() {
        collectingState = false;
        collector.set(ControlMode.PercentOutput, -Constants.Collector.COLLECT_PERCENT);
    }

    public void eject() {
        collectingState = false;
        collector.set(ControlMode.PercentOutput, -Constants.Collector.EJECT_PERCENT);
    }

    public void stop() {
        collectingState = false;
        collector.set(ControlMode.PercentOutput, 0);
    }

    public boolean getSensorState() {
        return collectorDebouncer.calculate(sensor.get());
//        return sensor.get();
    }

    public boolean getCollectingState() { return collectingState;}

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
