package org.team1540.robot2020.commands.shooter;

//public class ShooterSequence extends SequentialCommandGroup {
//    public ShooterSequence(Intake intake, Funnel funnel, Indexer indexer, Shooter shooter, Hood hood) {
//        addRequirements(intake, funnel, indexer, shooter, hood);
//        addCommands(
//                parallel(
//                        new ShooterSetVelocityContinuous(shooter, () -> 5000),
//                        new HoodSetPositionContinuous(hood, () -> -100)
//                ),
//                deadline(new WaitCommand(2),
//                        new InstantCommand(() -> indexer.setPercent(1)),
//                        new RunFunnel(funnel, indexer),
//                        new IntakeRun(intake)),
//                new RunCommand(() -> {
//                    intake.stop();
//                    funnel.stop();
//                    indexer.setPercent(0);
//                    shooter.disableMotors();
//                })
//        );
//    }
//}
