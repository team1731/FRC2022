package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * AUTONOMOUS TEST
 */
public class Z1_Move_Test extends _DelayableStrafingAutoMode {
	public Z1_Move_Test(DriveSubsystem m_robotDrive) {

		SequentialCommandGroup commandGroup = new SequentialCommandGroup(new WaitCommand(getInitialDelaySeconds()),

				createSwerveCommand(m_robotDrive, "MOVE TEST", TrajectoryDirection.FWD, TrajectoryHeading.DO_NOTHING,
						0, new double[][] { { 0.0, 0.0, 0.0 }, // initial pose
								{ 1.0, 0.0 }, // waypoint(s)
								{ 1.0, 1.0 },
								{ 1.0, 0.0, 0.0 } } // final pose
				));

		command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
	}
}
