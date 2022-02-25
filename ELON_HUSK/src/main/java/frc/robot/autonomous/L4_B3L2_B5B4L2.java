package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.commands.intake.RightIntakeCommand;
import frc.robot.commands.intake.RightIntakeJoyconCommand;
import frc.robot.commands.intake.RightStopCommand;
import frc.robot.commands.launch.LaunchBallCommand;
import frc.robot.commands.launch.LaunchBallCommandStart;
import frc.robot.commands.launch.LaunchBallCommandStop;
//import to create a new IntakeSubstem object
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;

/**
 * Starts at B3, launches 2, goes to B4 then B5 then launches 2
 */

public class L4_B3L2_B5B4L2 extends _DelayableStrafingAutoMode {



	private Pose2d _initPose;

	private IntakeSubsystem m_intake;
	private LaunchSubsystem m_launch;

	private double angleOffset;

	@Override
	public Pose2d getInitialPose() {
		return this._initPose;
	}

	@Override
	public double getAngleOffset() {
		return this.angleOffset;
	}


	public L4_B3L2_B5B4L2(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake2, LaunchSubsystem m_launch2) {

		String trajectoryJSON0 = "paths/output/L4-1.wpilib.json";
        String trajectoryJSON1 = "paths/output/L4-2.wpilib.json";
        String trajectoryJSON2 = "paths/output/L4-3.wpilib.json";
        this.m_intake = m_intake2;
		this.m_launch = m_launch2;

        Trajectory trajectory0 = new Trajectory();
        Trajectory trajectory1 = new Trajectory();
        Trajectory trajectory2 = new Trajectory();


        try {
            Path traj0Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON0);
            trajectory0 = TrajectoryUtil.fromPathweaverJson(traj0Path);
            Path traj1Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
            trajectory1 = TrajectoryUtil.fromPathweaverJson(traj1Path);
            Path traj2Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
            trajectory2 = TrajectoryUtil.fromPathweaverJson(traj2Path);

        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON0, ex.getStackTrace());
        }


		Pose2d unrotInitPose = trajectory0.getInitialPose();
		this._initPose = new Pose2d(unrotInitPose.getX(), unrotInitPose.getY(), Rotation2d.fromDegrees(-46.0));
		this.angleOffset = -46.0;   // this is the angle of the robot on the field. 

		SequentialCommandGroup commandGroup = new SequentialCommandGroup(

			new WaitCommand(getInitialDelaySeconds()),

			new ParallelCommandGroup(
			new RightIntakeCommand(m_intake).withTimeout(2),                                   // Turn on Intake
			createSwerveCommand(m_robotDrive, "L4_B3L2_B5B4L2", -46.0, trajectory0)), // Drive to first ball	
				

			new LaunchBallCommandStart(m_launch).withTimeout(4),
			new LaunchBallCommandStop(m_launch).withTimeout(0.1),

			new ParallelCommandGroup(
			createSwerveCommand(m_robotDrive, "L4_B3L2_B5B4L2", -46.0, trajectory1),  // Drive to second ball
			new RightIntakeCommand(m_intake).withTimeout(2)),
			
			new RightStopCommand(m_intake),
			createSwerveCommand(m_robotDrive, "L4_B3L2_B5B4L2", -46.0, trajectory2),  // Drive to first ball				
			new LaunchBallCommandStart(m_launch),
			new WaitCommand (3),			
			new LaunchBallCommandStop(m_launch)
					     
		);


        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0 ,0, false, false));
    }
}

