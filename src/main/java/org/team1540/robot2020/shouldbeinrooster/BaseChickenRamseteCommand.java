package org.team1540.robot2020.shouldbeinrooster;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import org.team1540.robot2020.subsystems.DriveTrain;

public class BaseChickenRamseteCommand extends RamseteCommand {
    public BaseChickenRamseteCommand(Trajectory trajectory, DriveTrain driveTrain) {
        super(
                trajectory,
                driveTrain::getPose,
                new RamseteController(DriveTrain.kRamseteB, DriveTrain.kRamseteZeta),
                new SimpleMotorFeedforward(
                        DriveTrain.ksVolts,
                        DriveTrain.kvVoltSecondsPerMeter,
                        DriveTrain.kaVoltSecondsSquaredPerMeter
                ),
                DriveTrain.kDriveKinematics,
                driveTrain::getWheelSpeeds,
                new PIDController(DriveTrain.kPDriveVel, 0, 0),
                new PIDController(DriveTrain.kPDriveVel, 0, 0),
                driveTrain::tankDriveVolts,
                driveTrain
        );
    }
}
