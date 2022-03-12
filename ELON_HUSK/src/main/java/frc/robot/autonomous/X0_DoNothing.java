package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;

/**
 * Starts at B3, launches 2, goes to B4 then B5 then launches 2
 */

public class X0_DoNothing extends _DelayableStrafingAutoMode {

	private Pose2d _initPose = new Pose2d();

	@Override
	public Pose2d getInitialPose() {
		return this._initPose;
	}

	public X0_DoNothing(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake2, LaunchSubsystem m_launch2) {
		command = new WaitCommand(10);
    }
}

