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

public class AutoThreeBall extends ParallelCommandGroup {

    public AutoThreeBall(DriveTrain driveTrain, Intake intake, Funnel funnel, Indexer indexer, Shooter shooter, Hood hood, Climber climber, LocalizationManager localizationManager, ChickenXboxController driverController, boolean zeroHood) {

        SmartDashboard.putNumber("AutoThreeBall/TimeBeforeDrive", 0);
        SmartDashboard.putNumber("AutoThreeBall/TimeBeforeShoot", 0);
        SmartDashboard.putBoolean("AutoThreeBall/DriveBeforeShoot", true);

        addCommands(
                new InstCommand(() -> {
                    climber.zero();
                    climber.setRatchet(Climber.RatchetState.DISENGAGED);
                }),
                new ConditionalCommand(
                        sequence(
                                parallel(
                                        new ConditionalCommand(
                                                new HoodZeroSequence(hood).asProxy(),
                                                new InstCommand(),
                                                () -> zeroHood
                                        ),
                                        sequence(
                                                new WaitSupplierCommand(() -> SmartDashboard.getNumber("AutoThreeBall/TimeBeforeDrive", 0)),
                                                new AutoThreeBallDrive(driveTrain, intake, funnel, indexer, shooter, hood, climber, localizationManager, driverController)
                                        )
                                ),
                                new WaitSupplierCommand(() -> SmartDashboard.getNumber("AutoThreeBall/TimeBeforeShoot", 0)),
                                new AutoShootNBalls(3, driveTrain, intake, funnel, indexer, shooter, hood, climber, localizationManager, driverController)
                        ),
                        sequence(
                                new HoodZeroSequence(hood),
                                new WaitSupplierCommand(() -> SmartDashboard.getNumber("AutoThreeBall/TimeBeforeShoot", 0)),
                                new AutoShootNBalls(3, driveTrain, intake, funnel, indexer, shooter, hood, climber, localizationManager, driverController),
                                new WaitSupplierCommand(() -> SmartDashboard.getNumber("AutoThreeBall/TimeBeforeDrive", 0)),
                                new AutoThreeBallDrive(driveTrain, intake, funnel, indexer, shooter, hood, climber, localizationManager, driverController)
                        ),
                        () -> SmartDashboard.getBoolean("AutoThreeBall/DriveBeforeShoot", true)
                )
        );
    }
}