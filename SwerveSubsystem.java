package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
    private static final double FL_OFFSET = Constants.SwerveConstants.FL_OFFSET;
    private static final double FR_OFFSET = Constants.SwerveConstants.FR_OFFSET;
    private static final double BL_OFFSET = Constants.SwerveConstants.BL_OFFSET;
    private static final double BR_OFFSET = Constants.SwerveConstants.BR_OFFSET;

    private final SwerveModule frontLeft = new SwerveModule(Constants.SwerveConstants.FL_DRIVE, Constants.SwerveConstants.FL_STEER, FL_OFFSET);
    private final SwerveModule frontRight = new SwerveModule(Constants.SwerveConstants.FR_DRIVE, Constants.SwerveConstants.FR_STEER, FR_OFFSET);
    private final SwerveModule backLeft = new SwerveModule(Constants.SwerveConstants.BL_DRIVE, Constants.SwerveConstants.BL_STEER, BL_OFFSET);
    private final SwerveModule backRight = new SwerveModule(Constants.SwerveConstants.BR_DRIVE, Constants.SwerveConstants.BR_STEER, BR_OFFSET);

    private final Pigeon2 gyro = new Pigeon2(Constants.SwerveConstants.PIGEON_2);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.SwerveConstants.WHEEL_BASE/2, Constants.SwerveConstants.TRACK_WIDTH/2),
        new Translation2d(Constants.SwerveConstants.WHEEL_BASE/2, -Constants.SwerveConstants.TRACK_WIDTH/2),
        new Translation2d(-Constants.SwerveConstants.WHEEL_BASE/2, Constants.SwerveConstants.TRACK_WIDTH/2),
        new Translation2d(-Constants.SwerveConstants.WHEEL_BASE/2, -Constants.SwerveConstants.TRACK_WIDTH/2)
    );

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        kinematics, getHeading(),
        new SwerveModulePosition[] { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition() },
        new Pose2d()
    );

    @Override
    public void periodic() {
        odometry.update(getHeading(), new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
        });
    }

    public void drive(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        if (Math.abs(speeds.vxMetersPerSecond) < 0.02 && Math.abs(speeds.vyMetersPerSecond) < 0.02 && Math.abs(speeds.omegaRadiansPerSecond) < 0.02) {
            frontLeft.stop(); frontRight.stop(); backLeft.stop(); backRight.stop();
            return;
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.MAX_SPEED);
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    public Pose2d getPose() { return odometry.getPoseMeters(); }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(getHeading(), new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
        }, pose);
    }

    public Rotation2d getHeading() { return Rotation2d.fromDegrees(-gyro.getYaw().getValueAsDouble()); }
    
    public void zeroGyro() { gyro.reset(); }
}