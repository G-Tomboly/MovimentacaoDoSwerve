package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem swerve = new SwerveSubsystem();
    private final Joystick joyDriver = new Joystick(Constants.JoystickConstants.joystickPort1);

    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2.5);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(2.5);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    public RobotContainer() {
        configureDefaultCommands();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new RunCommand(() -> {
            double xInput = applyDeadband(-joyDriver.getRawAxis(1));
            double yInput = applyDeadband(-joyDriver.getRawAxis(0));
            double rotInput = applyDeadband(-joyDriver.getRawAxis(2));

            swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedLimiter.calculate(xInput * Constants.SwerveConstants.MAX_SPEED),
                ySpeedLimiter.calculate(yInput * Constants.SwerveConstants.MAX_SPEED),
                rotLimiter.calculate(rotInput * Constants.SwerveConstants.MAX_ANGULAR_SPEED),
                swerve.getHeading()
            ));
        }, swerve));
    }

    private double applyDeadband(double value) {
        if (Math.abs(value) < 0.1) return 0.0;
        return Math.pow(value, 3);
    }

    public Command getAutonomousCommand() {
        return null; 
    }
}