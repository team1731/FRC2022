package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.IntakeSeqCommand;
import frc.robot.commands.ShootSeqCommandAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;

public class L1_EnemyPair_Front3 extends _DelayableStrafingAutoMode {
    public L1_EnemyPair_Front3(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequence,
            ShootClimbSubsystem m_shootclimb) {
                
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),

            // ENEMY PAIR
            new ParallelCommandGroup(
                createSwerveCommand(m_robotDrive, "BACKWARD TO ENEMY PAIR", TrajectoryDirection.REV, 
                                    TrajectoryHeading.UNROTATE, -31, new double[][]
                {{0.0, 0.0, 20},      // initial pose
                  {-1.49, -0.65},     // waypoint
                  {-2.4, -0.75},      // waypoint
                  {-2.3, -0.5},       // waypoint
                 {-2.66, -0.24,-20}}  // final pose
                ),
        
                new IntakeSeqCommand(m_intake, m_sequence, true).withTimeout(5)
            ),

            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),
                createSwerveCommand(m_robotDrive, "STRAFE TO SHOOT LOCATION 1", TrajectoryDirection.FWD, 
                                    TrajectoryHeading.UNROTATE, -15, new double[][]
                    {{-2.66, -0.24, -45}, // initial pose
                     {-0.18, -4.13, -45}} // final pose
                ),

            new WaitCommand(1),
            new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(2),
            new WaitCommand(getSecondaryDelaySeconds()),

            // FRONT 3
            new ParallelCommandGroup(
                createSwerveCommand(m_robotDrive, "BACKWARD TO FRONT 3", TrajectoryDirection.REV,
                                    TrajectoryHeading.MAINTAIN, 11, new double[][]
                {{-0.18, -4.13, -10}, // initial pose
                {-0.85, -3.8},        // waypoint(s)
                 {-1.9, -4.03},       // waypoint(s)
                 {-2.23, -3.27, 0}}   // final pose
                ),
        
                new IntakeSeqCommand(m_intake, m_sequence, true).withTimeout(4)
            ),

            new ParallelCommandGroup(
                new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),
                createSwerveCommand(m_robotDrive, "STRAFE TO SHOOT LOCATION 2", TrajectoryDirection.FWD, 
                                    TrajectoryHeading.UNROTATE, -15, new double[][]
                {{-2.23, -3.27, 20},   // initial pose
                 {-0.18, -4.13, 20}}  // final pose
                )
            ),

            new WaitCommand(1),
            new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(2)
        );

        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false)).andThen(() -> m_shootclimb.stopShooting());
    }
}