package team.gif.lib;

import team.gif.robot.Constants;

public enum shootParams {
    WALL(Constants.Wrist.SETPOINT_WALL_ABSOLUTE, Constants.Shooter.RPM_WALL, Constants.Shooter.RPM_MIN_WALL, Constants.Shooter.FF_WALL, Constants.Shooter.kP_WALL, Constants.Shooter.kI_WALL),
    NEAR(Constants.Wrist.SETPOINT_NEAR_ABSOLUTE, Constants.Shooter.RPM_NEAR, Constants.Shooter.RPM_MIN_NEAR, Constants.Shooter.FF_NEAR, Constants.Shooter.kP_NEAR, Constants.Shooter.kI_NEAR),
    MID(Constants.Wrist.SETPOINT_MID_ABSOLUTE, Constants.Shooter.RPM_MID, Constants.Shooter.RPM_MIN_MID, Constants.Shooter.FF_MID, Constants.Shooter.kP_MID, Constants.Shooter.kI_MID),
    FAR(Constants.Wrist.SETPOINT_FAR_ABSOLUTE, Constants.Shooter.RPM_FAR, Constants.Shooter.RPM_MIN_FAR, Constants.Shooter.FF_FAR, Constants.Shooter.kP_FAR, Constants.Shooter.kI_FAR),
    AMP(Constants.Wrist.SETPOINT_AMP_ABSOLUTE, Constants.Shooter.RPM_AMP, Constants.Shooter.RPM_MIN_AMP, Constants.Shooter.FF_AMP, Constants.Shooter.kP_AMP, Constants.Shooter.kI_AMP),
    AUTO_THREE(Constants.Wrist.SETPOINT_AUTO_THREE_ABSOLUTE, Constants.Shooter.RPM_AMP, Constants.Shooter.RPM_MIN_AMP, Constants.Shooter.FF_AMP, Constants.Shooter.kP_AMP, Constants.Shooter.kI_AMP),
    AUTO_FAR(Constants.Wrist.SETPOINT_AUTO_FAR_ABSOLUTE, Constants.Shooter.RPM_FAR, Constants.Shooter.RPM_MIN_FAR, Constants.Shooter.FF_FAR, Constants.Shooter.kP_FAR, Constants.Shooter.kI_FAR);


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
