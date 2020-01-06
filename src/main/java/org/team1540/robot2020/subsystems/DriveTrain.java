package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.rooster.wrappers.ChickenTalon;
import org.team1540.rooster.wrappers.ChickenVictor;

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
    private final DifferentialDriveKinematics kDriveKinematics =  new DifferentialDriveKinematics(kTrackwidthMeters);

    private Talon driveLeftA = new Talon(13);
    private Victor driveLeftB = new Victor(12);
    private Victor driveLeftC = new Victor(11);

    private Talon driveRightA = new Talon(1);
    private Victor driveRightB = new Victor(2);
    private Victor driveRightC = new Victor(3);

    private SpeedControllerGroup driveLeft = new SpeedControllerGroup(driveLeftA, driveLeftB, driveLeftC);
    private SpeedControllerGroup driveRight = new SpeedControllerGroup(driveRightA, driveRightB, driveRightC);

    private DifferentialDrive drive = new DifferentialDrive(driveLeft, driveRight);

    public DriveTrain() {
//        driveLeftB.follow(driveLeftA);
//        driveLeftC.follow(driveLeftA);
//
//        driveRightB.follow(driveRightA);
//        driveRightC.follow(driveRightA);

//        resetEncoders();
    }

//    @Override
//    public void periodic() {
//        SmartDashboard.putNumber("drive/leftEncoder", driveLeftA.getSelectedSensorPosition());
//        SmartDashboard.putNumber("drive/rightEncoder", driveRightA.getSelectedSensorPosition());
//    }
//
//    public void setThrottle(double left, double right) {
//        driveLeftA.set(ControlMode.PercentOutput, -left);
//        driveRightA.set(ControlMode.PercentOutput, right);
//    }
//
//    public void resetEncoders() {
//        driveLeftA.setSelectedSensorPosition(0);
//        driveRightA.setSelectedSensorPosition(0);
//    }
}
