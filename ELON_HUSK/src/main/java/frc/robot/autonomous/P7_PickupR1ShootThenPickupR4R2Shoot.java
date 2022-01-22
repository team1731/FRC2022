package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Taxi, get R1, Shoot 2, get R4, get R2, Shoot 2. Used when at least one cannot get 2nd cargo. *This is ideal if one cannot shoot.*
 * Starting Position: TOP
 */
public class P7_PickupR1ShootThenPickupR4R2Shoot extends _DelayableStrafingAutoMode {

	private final String trajectory1JSON = AutoConstants.kPATH + ".wpilib.json";

	private Pose2d _initPose;

	@Override
	public Pose2d getInitialPose() {
		return this._initPose;
	}

	public P7_PickupR1ShootThenPickupR4R2Shoot(DriveSubsystem m_robotDrive) {

		//Resolve trajectories
		Trajectory trajectory1 = new Trajectory();
		
		try {
			Path trajectory1FilePath = Filesystem.getDeployDirectory().toPath().resolve(trajectory1JSON);
			trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectory1FilePath);
		} catch (IOException ioEx){
			DriverStation.reportError("AUTO FATAL: UNABLE TO OPEN ONE OR ALL TRAJECTORIES!!!", ioEx.getStackTrace());
		}

		Pose2d unrotInitPose = trajectory1.getInitialPose();
		this._initPose = new Pose2d(unrotInitPose.getX(), unrotInitPose.getY(), Rotation2d.fromDegrees(90));

		//PathWeaver uses absolute field coords, reset odometry
		m_robotDrive.resetOdometry(this._initPose);
		SequentialCommandGroup commandGroup = new SequentialCommandGroup(
			new WaitCommand(getInitialDelaySeconds()),
			createSwerveCommand(m_robotDrive, "P7_PickupR1ShootThenPickupR4R2Shoot", 0, trajectory1)
		);

		command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
	}
}
