package org.team1540.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.team1540.rooster.wrappers.ChickenTalon;
import org.team1540.rooster.wrappers.ChickenVictor;

public class DriveTrain implements Subsystem {
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

    private ChickenTalon driveLeftA = new ChickenTalon(13);
    private ChickenVictor driveLeftB = new ChickenVictor(12);
    private ChickenVictor driveLeftC = new ChickenVictor(11);

    private ChickenTalon driveRightA = new ChickenTalon(1);
    private ChickenVictor driveRightB = new ChickenVictor(2);
    private ChickenVictor driveRightC = new ChickenVictor(3);

    public DriveTrain() {
        driveLeftB.follow(driveLeftA);
        driveLeftC.follow(driveLeftA);

        driveRightB.follow(driveRightA);
        driveRightC.follow(driveRightA);

        resetEncoders();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("drive/leftEncoder", driveLeftA.getSelectedSensorPosition());
        SmartDashboard.putNumber("drive/rightEncoder", driveRightA.getSelectedSensorPosition());
    }

    public void setThrottle(double left, double right) {
        driveLeftA.set(ControlMode.PercentOutput, -left);
        driveRightA.set(ControlMode.PercentOutput, right);
    }

    public void resetEncoders() {
        driveLeftA.setSelectedSensorPosition(0);
        driveRightA.setSelectedSensorPosition(0);
    }
}
