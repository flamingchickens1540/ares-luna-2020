package org.team1540.robot2020.shouldbeinrooster;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import org.team1540.robot2020.subsystems.DriveTrain;

public class ChickenRamseteCommand extends RamseteCommand {
    public ChickenRamseteCommand(Trajectory trajectory, DriveTrain driveTrain) {
        super(
                trajectory,
                driveTrain::getPose,
                new RamseteController(driveTrain.kRamseteB, driveTrain.kRamseteZeta),
                new SimpleMotorFeedforward(
                        driveTrain.ksVolts,
                        driveTrain.kvVoltSecondsPerMeter,
                        driveTrain.kaVoltSecondsSquaredPerMeter
                ),
                driveTrain.kDriveKinematics,
                driveTrain::getWheelSpeeds,
                new PIDController(driveTrain.kPDriveVel, 0, 0),
                new PIDController(driveTrain.kPDriveVel, 0, 0),
                driveTrain::tankDriveVolts,
                driveTrain
        );
    }
}
