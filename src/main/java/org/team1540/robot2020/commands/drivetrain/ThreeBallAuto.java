package org.team1540.robot2020.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2020.LocalizationManager;
import org.team1540.robot2020.commands.climber.Climber;
import org.team1540.robot2020.commands.funnel.Funnel;
import org.team1540.robot2020.commands.hood.Hood;
import org.team1540.robot2020.commands.hood.HoodZeroSequence;
import org.team1540.robot2020.commands.indexer.Indexer;
import org.team1540.robot2020.commands.intake.Intake;
import org.team1540.robot2020.commands.shooter.Shooter;
import org.team1540.robot2020.utils.ChickenXboxController;
import org.team1540.robot2020.utils.InstCommand;
import org.team1540.robot2020.utils.WaitSupplierCommand;

public class ThreeBallAuto extends ParallelCommandGroup {

    public ThreeBallAuto(DriveTrain driveTrain, Intake intake, Funnel funnel, Indexer indexer, Shooter shooter, Hood hood, Climber climber, LocalizationManager localizationManager, ChickenXboxController driverController) {

        SmartDashboard.putNumber("ThreeBallAuto/TimeBeforeDrive", 0);
        SmartDashboard.putNumber("ThreeBallAuto/TimeBeforeShoot", 0);
        SmartDashboard.putBoolean("ThreeBallAuto/DriveBeforeShoot", true);

        addCommands(
                new InstCommand(() -> {
                    climber.zero();
                    climber.setRatchet(Climber.RatchetState.DISENGAGED);
                }),
                new ConditionalCommand(
                        sequence(
                                parallel(
                                        new HoodZeroSequence(hood),
                                        sequence(
                                                new WaitSupplierCommand(() -> SmartDashboard.getNumber("ThreeBallAuto/TimeBeforeDrive", 0)),
                                                new ThreeBallAutoDrive(driveTrain, intake, funnel, indexer, shooter, hood, climber, localizationManager, driverController)
                                        )
                                ),
                                new WaitSupplierCommand(() -> SmartDashboard.getNumber("ThreeBallAuto/TimeBeforeShoot", 0)),
                                new AutoShootThreeBalls(driveTrain, intake, funnel, indexer, shooter, hood, climber, localizationManager, driverController)
                        ),
                        sequence(
                                new HoodZeroSequence(hood),
                                new WaitSupplierCommand(() -> SmartDashboard.getNumber("ThreeBallAuto/TimeBeforeShoot", 0)),
                                new AutoShootThreeBalls(driveTrain, intake, funnel, indexer, shooter, hood, climber, localizationManager, driverController),
                                new WaitSupplierCommand(() -> SmartDashboard.getNumber("ThreeBallAuto/TimeBeforeDrive", 0)),
                                new ThreeBallAutoDrive(driveTrain, intake, funnel, indexer, shooter, hood, climber, localizationManager, driverController)
                        ),
                        () -> SmartDashboard.getBoolean("ThreeBallAuto/DriveBeforeShoot", true)
                )
        );
    }
}