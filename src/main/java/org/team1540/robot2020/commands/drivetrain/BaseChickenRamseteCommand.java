package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.RamseteConfig;
import org.team1540.robot2020.utils.RamseteCommand;

import java.util.function.Supplier;

public class BaseChickenRamseteCommand extends RamseteCommand {
    public BaseChickenRamseteCommand(Supplier<Trajectory> trajectory, LocalizationManager localizationManager, DriveTrain driveTrain) {
        super(
                trajectory,
                localizationManager::odometryGetPose,
                new RamseteController(RamseteConfig.kRamseteB, RamseteConfig.kRamseteZeta),
                new SimpleMotorFeedforward(
                        RamseteConfig.ksVolts,
                        RamseteConfig.kvVoltSecondsPerMeter,
                        RamseteConfig.kaVoltSecondsSquaredPerMeter
                ),
                RamseteConfig.kDriveKinematics,
                driveTrain::getWheelSpeeds,
                new PIDController(RamseteConfig.kPDriveVel, 0, 0),
                new PIDController(RamseteConfig.kPDriveVel, 0, 0),
                driveTrain::setVoltage,
                driveTrain
        );
    }
}