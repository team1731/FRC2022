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
 * Shoot, Taxi, get R1, R2, or R3, Shoot. Used when all can shoot 2, someone can get R4 better than us
 * Starting Position: ANY
 */
public class L1_ShootThenPickupRandomShoot extends _DelayableStrafingAutoMode {

	private final String trajectory1JSON = AutoConstants.kPATH + ".wpilib.json";

	private Pose2d _initPose;

	@Override
	public Pose2d getInitialPose() {
		return this._initPose;
	}

	public L1_ShootThenPickupRandomShoot(DriveSubsystem m_robotDrive) {

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
			createSwerveCommand(m_robotDrive, "P1_ShootTaxiThenPickupShoot", 0, trajectory1)
		);

		command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
	}
}
