package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2020.shouldbeinrooster.Encoder;
import org.team1540.robot2020.shouldbeinrooster.NavX;
import org.team1540.robot2020.shouldbeinrooster.TalonEncoder;

public class DriveTrain extends SubsystemBase {
    private final double kS = 0.669;
    private final double kV = 2.76;
    private final double kA = 0.662;
    private final double kP = 0.5;
    private final double kI = 0;
    private final double kD = 0;
    private final double kRamseteB = 2;
    private final double kRamseteZeta = 0.7;
    private final double kMaxSpeedMetersPerSecond = 3;
    private final double kMaxAccelerationMetersPerSecondSquared = 3;
    private final double kPDriveVel = 19.3;
    private final double kTrackwidthMeters = 0.761388065;

    private final double kWheelCircumference = 0.4863499587;
    private final double kEncoderPPR = 512;
    private final double kEncoderDistancePerPulse = kWheelCircumference / kEncoderPPR;

    private final DifferentialDriveKinematics kDriveKinematics =  new DifferentialDriveKinematics(kTrackwidthMeters);

    private WPI_TalonSRX driveLeftA = new WPI_TalonSRX(13);
    private WPI_VictorSPX driveLeftB = new WPI_VictorSPX(12);
    private WPI_VictorSPX driveLeftC = new WPI_VictorSPX(11);

    private WPI_TalonSRX driveRightA = new WPI_TalonSRX(1);
    private WPI_VictorSPX driveRightB = new WPI_VictorSPX(2);
    private WPI_VictorSPX driveRightC = new WPI_VictorSPX(3);

    private SpeedControllerGroup leftMotors = new SpeedControllerGroup(driveLeftA, driveLeftB, driveLeftC);
    private SpeedControllerGroup rightMotors = new SpeedControllerGroup(driveRightA, driveRightB, driveRightC);

    private DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    private Encoder leftEncoder = new TalonEncoder(driveLeftA);
    private Encoder rightEncoder = new TalonEncoder(driveRightA);

    // The gyro sensor
    private final NavX navx = new NavX(Port.kMXP);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry;

    private boolean kGyroReversed = false;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveTrain() {
        leftMotors.setInverted(true);
        rightMotors.setInverted(true);

        // Sets the distance per pulse for the encoders
        leftEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
        rightEncoder.setDistancePerPulse(kEncoderDistancePerPulse);

        resetEncoders();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(),
            rightEncoder.getDistance());

        SmartDashboard.putNumber("drive/leftEncoderDistance", leftEncoder.getDistance());
        SmartDashboard.putNumber("drive/rightEncoderDistance", rightEncoder.getDistance());

        SmartDashboard.putNumber("drive/leftEncoderSpeed", leftEncoder.getRate());
        SmartDashboard.putNumber("drive/rightEncoderSpeed", rightEncoder.getRate());

        SmartDashboard.putNumber("drive/leftEncoderTicks", driveLeftA.getSelectedSensorPosition());
        SmartDashboard.putNumber("drive/rightEncoderTicks", driveRightA.getSelectedSensorPosition());

        Pose2d poseMeters = odometry.getPoseMeters();
        Translation2d postTranslation = poseMeters.getTranslation();
        SmartDashboard.putNumber("drive/odometry/X", postTranslation.getX());
        SmartDashboard.putNumber("drive/odometry/Y", postTranslation.getY());
        SmartDashboard.putNumber("drive/odometry/rotationDegrees", poseMeters.getRotation().getDegrees());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(-rightVolts);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        driveLeftA.setSelectedSensorPosition(0);
        driveRightA.setSelectedSensorPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        navx.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(navx.getAngle(), 360) * (kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return navx.getRate() * (kGyroReversed ? -1.0 : 1.0);
    }
}
