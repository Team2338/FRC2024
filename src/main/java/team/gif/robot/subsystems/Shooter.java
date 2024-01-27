package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.Constants;
import team.gif.robot.Robot;
import team.gif.robot.RobotMap;

public class Shooter extends SubsystemBase {
    public static CANSparkMax shooter;

    public Shooter() {
        shooter = new CANSparkMax(RobotMap.SHOOTER, CANSparkLowLevel.MotorType.kBrushless);

        shooter.getPIDController().setFF(Constants.Shooter.ff_gain);
        shooter.getPIDController().setP(Constants.Shooter.p_gain);
        shooter.getPIDController().setD(Constants.Shooter.d_gain);
    }

    public void setShooter(double volt) {
        shooter.setVoltage(volt);
    }
}
