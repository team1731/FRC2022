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
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.commands.intake.RightIntakeCommand;
import frc.robot.commands.intake.RightStopCommand;
import frc.robot.commands.launch.LaunchBallCommandStart;
import frc.robot.commands.launch.LaunchBallCommandStop;
import frc.robot.commands.launch.LaunchCommandStart;
import frc.robot.commands.launch.LaunchCommandStop;
//import to create a new IntakeSubstem object
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;

/**
 * Starts at B3, launches 2, goes to B4 then B5 then launches 2
 */

public class D2_B3X2 extends _DelayableStrafingAutoMode {



	private Pose2d _initPose;

	private IntakeSubsystem m_intake;
	private LaunchSubsystem m_launch;



	@Override
	public Pose2d getInitialPose() {
		return this._initPose;
	}




	public D2_B3X2(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake2, LaunchSubsystem m_launch2) {

		String trajectoryJSON0 = "paths/output/L4-1.wpilib.json";
		String trajectoryJSON1 = "paths/output/L1-2.wpilib.json";
		String trajectoryJSON2 = "paths/output/D2-2.wpilib.json";
		String trajectoryJSON3 = "paths/output/D2-1.wpilib.json";
        this.m_intake = m_intake2;
		this.m_launch = m_launch2;

        Trajectory trajectory0 = new Trajectory();
		Trajectory trajectory1 = new Trajectory();
		Trajectory trajectory2 = new Trajectory();
		Trajectory trajectory3 = new Trajectory();


        try {
            Path traj0Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON0);
            trajectory0 = TrajectoryUtil.fromPathweaverJson(traj0Path);

			Path traj1Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
            trajectory1 = TrajectoryUtil.fromPathweaverJson(traj1Path);

			Path traj2Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
            trajectory2 = TrajectoryUtil.fromPathweaverJson(traj2Path);

			Path traj3Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON3);
            trajectory3 = TrajectoryUtil.fromPathweaverJson(traj3Path);

        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON0, ex.getStackTrace());
        }


		Pose2d unrotInitPose = trajectory0.getInitialPose();
		this._initPose = new Pose2d(unrotInitPose.getX(), unrotInitPose.getY(), Rotation2d.fromDegrees(-46.0));


		SequentialCommandGroup commandGroup = new SequentialCommandGroup(
			new WaitCommand(getInitialDelaySeconds()),
            // new LaunchCommandStart(m_launch,0.375,true).withTimeout(1.5),
			// new LaunchBallCommandStart(m_launch),
			// new LaunchCommandStart(m_launch,0.375,true).withTimeout(1),
			// new LaunchBallCommandStop(m_launch),
			new LaunchCommandStart(m_launch,0.42,true).raceWith(new RightIntakeCommand(m_intake)),
			new LaunchCommandStart(m_launch,0.42,true).raceWith(createSwerveCommand(m_robotDrive, "L4_B3L2_B5B4L2", -40.0, trajectory0,false)), // Drive to first ball	
			new LaunchCommandStart(m_launch,.42,true).raceWith(createSwerveCommand(m_robotDrive, "L4_B3L2_B5B4L2", -40.0, trajectory1,false)), // Drive to first ball
            new LaunchBallCommandStart(m_launch),
			new LaunchCommandStart(m_launch,0.42,true).withTimeout(2),
			new LaunchBallCommandStop(m_launch),
			new LaunchCommandStart(m_launch,.4,true).raceWith(createSwerveCommand(m_robotDrive, "L4_B3L2_B5B4L2", -40.0, trajectory2,false)), // Drive to first ball
			new LaunchCommandStart(m_launch,.4,true).raceWith(createSwerveCommand(m_robotDrive, "L4_B3L2_B5B4L2", 180.0, trajectory3,false)), // Drive to first ball
			new LaunchBallCommandStart(m_launch),
			new LaunchCommandStart(m_launch,0.4,true).withTimeout(2),
			new LaunchCommandStop(m_launch),
			new RightStopCommand(m_intake)
					     
		);


        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0 ,0, false, false))
			.andThen(() -> m_robotDrive.setAngleAdjustment(-1*this._initPose.getRotation().getDegrees()));
    }
}

