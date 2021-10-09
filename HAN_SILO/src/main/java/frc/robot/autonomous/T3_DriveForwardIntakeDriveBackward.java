package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.commands.IntakeSeqCommand;

public class T3_DriveForwardIntakeDriveBackward extends _DelayableStrafingAutoMode {
    public T3_DriveForwardIntakeDriveBackward(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake,
            SequencerSubsystem m_sequence, ShootClimbSubsystem m_shootclimb) {

        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),

            new ParallelCommandGroup(
                createSwerveCommand(m_robotDrive, "FORWARD 1 METER", TrajectoryDirection.FWD, 
                                TrajectoryHeading.DO_NOTHING, 0, new double[][]
                    {{0, 0, 0},    // initial pose
                    {0.5, 0},     // waypoint(s)
                    {1, 0, 0}}    // final pose
                ),

                new IntakeSeqCommand(m_intake, m_sequence, true).withTimeout(4)
            ),
            // SHOOT 3
            //new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),

            //new WaitCommand(3),

            //new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(2),

            //new WaitCommand(getSecondaryDelaySeconds()),

            createSwerveCommand(m_robotDrive, "BACKWARD 1 METER", TrajectoryDirection.REV, 
                                TrajectoryHeading.DO_NOTHING, 0, new double[][]
                {{1, 0, 0},    // initial pose
                {0.5, 0},     // waypoint(s)
                {0, 0, 0}}    // final pose
            )
        );

        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false)).andThen(() -> m_shootclimb.stopShooting());
    }
}