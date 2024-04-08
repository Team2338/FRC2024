package team.gif.lib;

import team.gif.robot.Constants;

public enum shootParams {
    FAR3(Constants.Wrist.SETPOINT_FAR3_ABSOLUTE, Constants.Shooter.RPM_FAR3, Constants.Shooter.RPM_MIN_FAR3, Constants.Shooter.FF_FAR3, Constants.Shooter.kP_FAR3, Constants.Shooter.kI_FAR3),
    FAR2(Constants.Wrist.SETPOINT_FAR2_ABSOLUTE, Constants.Shooter.RPM_FAR2, Constants.Shooter.RPM_MIN_FAR2, Constants.Shooter.FF_FAR2, Constants.Shooter.kP_FAR2, Constants.Shooter.kI_FAR2),
    FAR(Constants.Wrist.SETPOINT_FAR_ABSOLUTE, Constants.Shooter.RPM_FAR, Constants.Shooter.RPM_MIN_FAR, Constants.Shooter.FF_FAR, Constants.Shooter.kP_FAR, Constants.Shooter.kI_FAR),
    MIDMIDFAR(Constants.Wrist.SETPOINT_MIDMIDFAR_ABSOLUTE, Constants.Shooter.RPM_MIDMIDFAR, Constants.Shooter.RPM_MIN_MIDMIDFAR, Constants.Shooter.FF_MIDMIDFAR, Constants.Shooter.kP_MIDMIDFAR, Constants.Shooter.kI_MIDMIDFAR),
    MIDFAR(Constants.Wrist.SETPOINT_MIDFAR_ABSOLUTE, Constants.Shooter.RPM_MIDFAR, Constants.Shooter.RPM_MIN_MIDFAR, Constants.Shooter.FF_MIDFAR, Constants.Shooter.kP_MIDFAR, Constants.Shooter.kI_MIDFAR),
    MID(Constants.Wrist.SETPOINT_MID_ABSOLUTE, Constants.Shooter.RPM_MID, Constants.Shooter.RPM_MIN_MID, Constants.Shooter.FF_MID, Constants.Shooter.kP_MID, Constants.Shooter.kI_MID),
    MIDDLE(Constants.Wrist.SETPOINT_MIDDLE_ABSOLUTE, Constants.Shooter.RPM_MIDDLE, Constants.Shooter.RPM_MIN_MIDDLE, Constants.Shooter.FF_MIDDLE, Constants.Shooter.kP_MIDDLE, Constants.Shooter.kI_MIDDLE),
    NEAR(Constants.Wrist.SETPOINT_NEAR_ABSOLUTE, Constants.Shooter.RPM_NEAR, Constants.Shooter.RPM_MIN_NEAR, Constants.Shooter.FF_NEAR, Constants.Shooter.kP_NEAR, Constants.Shooter.kI_NEAR),
    CLOSE(Constants.Wrist.SETPOINT_CLOSE_ABSOLUTE, Constants.Shooter.RPM_CLOSE, Constants.Shooter.RPM_MIN_CLOSE, Constants.Shooter.FF_CLOSE, Constants.Shooter.kP_CLOSE, Constants.Shooter.kI_CLOSE),
    WALL(Constants.Wrist.SETPOINT_WALL_ABSOLUTE, Constants.Shooter.RPM_WALL, Constants.Shooter.RPM_MIN_WALL, Constants.Shooter.FF_WALL, Constants.Shooter.kP_WALL, Constants.Shooter.kI_WALL),

    TRAP(Constants.Wrist.SETPOINT_TRAP_ABSOLUTE, Constants.Shooter.RPM_TRAP, Constants.Shooter.RPM_MIN_TRAP, Constants.Shooter.FF_TRAP, Constants.Shooter.kP_TRAP, Constants.Shooter.kI_TRAP),
    AMP(Constants.Wrist.SETPOINT_AMP_ABSOLUTE, Constants.Shooter.RPM_AMP, Constants.Shooter.RPM_MIN_AMP, Constants.Shooter.FF_AMP, Constants.Shooter.kP_AMP, Constants.Shooter.kI_AMP),
    AUTO(0,0,0,0,0,0);

    private double wristAngle;
    private double shooterRPM;
    private double minimumRPM;
    private double FF;
    private double kP;
    private double kI;

    shootParams(double wristAngle, double shooterRPM, double minimumRPM, double FF, double kP, double kI) {
        this.wristAngle = wristAngle;
        this.shooterRPM = shooterRPM;
        this.minimumRPM = minimumRPM;
        this.FF = FF;
        this.kP = kP;
        this.kI = kI;
    }

    public double getWristAngle() {
        return this.wristAngle;
    }
    public double getShooterRPM() {
        return this.shooterRPM;
    }
    public double getMinimumRPM() {
        return this.minimumRPM;
    }
    public double getFF() {
        return this.FF;
    }
    public double getP() {
        return this.kP;
    }
    public double getI() {
        return this.kI;
    }
}
