package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final PIDController steerPID;
    private static final String kP_KEY = "Swerve kP";
    private static final String kI_KEY = "Swerve kI";
    private static final String kD_KEY = "Swerve kD";

    public SwerveModule(int driveID, int steerID, double angleOffsetRot) {
        steerPID = new PIDController(
            Constants.SwerveConstants.STEER_kP, 
            Constants.SwerveConstants.STEER_kI, 
            Constants.SwerveConstants.STEER_kD
        );

        driveMotor = new TalonFX(driveID);
        steerMotor = new TalonFX(steerID);

        CurrentLimitsConfigs limits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(35)
            .withSupplyCurrentLimitEnable(true);
        
        driveMotor.getConfigurator().apply(limits);
        steerMotor.getConfigurator().apply(limits);

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        steerMotor.setNeutralMode(NeutralModeValue.Brake);

        SmartDashboard.setDefaultNumber(kP_KEY, Constants.SwerveConstants.STEER_kP);
        SmartDashboard.setDefaultNumber(kI_KEY, Constants.SwerveConstants.STEER_kI);
        SmartDashboard.setDefaultNumber(kD_KEY, Constants.SwerveConstants.STEER_kD);

        steerMotor.getPosition().waitForUpdate(0.25);
        double currentPos = steerMotor.getPosition().getValueAsDouble();
        steerMotor.setPosition(currentPos - angleOffsetRot);

        steerPID.enableContinuousInput(-Math.PI, Math.PI);
        steerPID.setTolerance(Math.toRadians(1.0));
    }

    private void updatePIDFromDashboard() {
        double p = SmartDashboard.getNumber(kP_KEY, Constants.SwerveConstants.defaultValue);
        double i = SmartDashboard.getNumber(kI_KEY, Constants.SwerveConstants.defaultValue);
        double d = SmartDashboard.getNumber(kD_KEY, Constants.SwerveConstants.defaultValue);
        if (p != steerPID.getP()) steerPID.setP(p);
        if (i != steerPID.getI()) steerPID.setI(i);
        if (d != steerPID.getD()) steerPID.setD(d);
    }

    public double getSteerAngleRad() {
        return steerMotor.getPosition().getValueAsDouble() * 2.0 * Math.PI;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getVelocity().getValueAsDouble() * Constants.SwerveConstants.MAX_SPEED,
            new Rotation2d(getSteerAngleRad())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getPosition().getValueAsDouble(),
            new Rotation2d(getSteerAngleRad())
        );
    }

    @SuppressWarnings("deprecation")
    public void setState(SwerveModuleState state) {
        updatePIDFromDashboard(); 
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return; 
        }
        state = SwerveModuleState.optimize(state, new Rotation2d(getSteerAngleRad()));
        double steerOutput = steerPID.calculate(getSteerAngleRad(), state.angle.getRadians());
        double driveOutput = state.speedMetersPerSecond / Constants.SwerveConstants.MAX_SPEED;
        driveMotor.set(driveOutput);
        steerMotor.set(steerOutput);
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}