package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Taxi. Used when everything is broken.
 * Starting Position: ANY
 */
public class T1_Taxi extends _DelayableStrafingAutoMode {
	public T1_Taxi(DriveSubsystem m_robotDrive) {

		SequentialCommandGroup commandGroup = new SequentialCommandGroup(new WaitCommand(getInitialDelaySeconds()),

				createSwerveCommand(m_robotDrive, "MOVE FORWARD", TrajectoryDirection.FWD, TrajectoryHeading.DO_NOTHING,
						0, new double[][] { { 0.0, 0.0, 0.0 }, // initial pose
								{ 0.5, 0.0 }, // waypoint(s)
								{ 1.0, 0.0, 0.0 } } // final pose
				));

		command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
	}
}
