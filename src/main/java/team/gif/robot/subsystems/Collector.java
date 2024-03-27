package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.RobotMap;

public class Collector extends SubsystemBase {

    public static TalonSRX motor;
    public static boolean collectingState;

    public Collector(){
        motor = new TalonSRX(RobotMap.COLLECTOR_ID);
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);

        collectingState = false;
    }

    public void collect() {
        collectingState = true;
        motor.set(ControlMode.PercentOutput, Constants.Collector.COLLECT_PERCENT);
    }

    public void reverse() {
        collectingState = false;
        motor.set(ControlMode.PercentOutput, -Constants.Collector.COLLECT_PERCENT);
    }

    public void eject() {
        collectingState = false;
        motor.set(ControlMode.PercentOutput, -Constants.Collector.EJECT_PERCENT);
    }

    public void stop() {
        collectingState = false;
        motor.set(ControlMode.PercentOutput, 0);
    }

    public boolean getCollectingState() { return collectingState;}

    /**
     * Use the current draw of the collector to determine if there is a note in the collector
     * @return
     */
    public boolean getPreSensorState() {
        return motor.getStatorCurrent() > Constants.Collector.PRE_SENSOR_AMPS;
    }

    public double getMotorTemp() {
        return motor.getTemperature();
    }

    public boolean isStageOneMotorCool() {
        return getMotorTemp() < Constants.MotorTemps.COLLECTOR_MOTOR_TEMP;
    }
}
