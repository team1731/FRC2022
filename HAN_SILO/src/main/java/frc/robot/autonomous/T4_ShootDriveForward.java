package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootSeqCommandAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;


public class T4_ShootDriveForward extends _DelayableStrafingAutoMode {
    public T4_ShootDriveForward(DriveSubsystem m_robotDrive,
            SequencerSubsystem m_sequence, ShootClimbSubsystem m_shootclimb) {

        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),

            new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),

            new WaitCommand(3),

            new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(2),

            new WaitCommand(getSecondaryDelaySeconds()),

            createSwerveCommand(m_robotDrive, "FORWARD 1 METER", TrajectoryDirection.FWD, 
                                TrajectoryHeading.DO_NOTHING, 0, new double[][]
                {{0, 0, 0},    //initial pose
                {0.5, 0},     // waypoint(s)
                {1, 0, 0}}    // final pose
            )
        );

        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false)).andThen(() -> m_shootclimb.stopShooting());
    }
}