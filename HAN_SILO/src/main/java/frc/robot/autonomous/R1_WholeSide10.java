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

public class R1_WholeSide10 extends _DelayableStrafingAutoMode {
	public R1_WholeSide10(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequence,
			ShootClimbSubsystem m_shootclimb) {

		SequentialCommandGroup commandGroup = new SequentialCommandGroup(new WaitCommand(getInitialDelaySeconds()),

				// BACKWARD TO BUDDY 2
				new ParallelCommandGroup(createSwerveCommand(m_robotDrive, "BACKWARD TO BUDDY 2",
						TrajectoryDirection.REV, TrajectoryHeading.UNROTATE, 70, new double[][] { { 0, 0, 0 }, // initial
																												// pose
								{ -2, -3.11 }, // waypoint(s)
								{ -3.93, -3.11 }, { -7.06, -3.01, 0 } } // final pose
				), new IntakeSeqCommand(m_intake, m_sequence).withTimeout(4)),

				new ParallelCommandGroup(createSwerveCommand(m_robotDrive, "STRAFE TO PICKUP BUDDY 2",
						TrajectoryDirection.REV, TrajectoryHeading.MAINTAIN, 65, new double[][] { { -2.14, -3.4, 65 }, // initial
																														// pose
								{ -2.11, -3.53 }, // waypoint(s)
								{ -1.79, -4.08, 70 } } // final pose
				), new IntakeSeqCommand(m_intake, m_sequence).withTimeout(4)),

				new ParallelCommandGroup(new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),
						createSwerveCommand(m_robotDrive, "STRAFE TO SHOOT LOCATION 1", TrajectoryDirection.FWD,
								TrajectoryHeading.UNROTATE, 0, new double[][] { { -2.14, -3.4, 65 }, // initial pose
										{ -0.89, -4.24 }, // waypoint(s)
										{ -0.54, -4.74, 50 } } // final pose
						)),

				new WaitCommand(1), new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(1),
				new WaitCommand(getSecondaryDelaySeconds()),

				// TRENCH 3
				new ParallelCommandGroup(createSwerveCommand(m_robotDrive, "BACKWARD TO TRENCH 3",
						TrajectoryDirection.REV, TrajectoryHeading.UNROTATE, 0, new double[][] { { 0, 0, 0 }, // initial
																												// pose
								{ -2, -3.11 }, // waypoint(s)
								{ -3.93, -3.11 }, { -7.06, -3.01, 0 } } // final pose
				), new IntakeSeqCommand(m_intake, m_sequence).withTimeout(4)),

				new ParallelCommandGroup(new InstantCommand(m_shootclimb::enableShooting, m_shootclimb).withTimeout(4),
						createSwerveCommand(m_robotDrive, "STRAFE TO SHOOT LOCATION 2", TrajectoryDirection.FWD,
								TrajectoryHeading.UNROTATE, 0, new double[][] { { -2.14, -3.4, 65 }, // initial pose
										{ -0.89, -4.24 }, // waypoint(s)
										{ -0.54, -4.74, 50 } } // final pose
						)),

				new WaitCommand(1), new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(1));

		// Run path following command, then stop at the end.
		command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false))
				.andThen(() -> m_shootclimb.stopShooting());
	}
}