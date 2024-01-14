package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class InPuts extends SubsystemBase {
    public static WPI_TalonSRX motor;
    public static TalonFX fxMotor;

    public InPuts() {
        fxMotor = new TalonFX(0);
        fxMotor.
    }

    public StatusSignal<Double> getTemp() {
        return fxMotor.getDeviceTemp();
    }
}
