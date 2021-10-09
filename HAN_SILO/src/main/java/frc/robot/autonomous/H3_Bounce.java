package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.DriveSubsystem;

// This is the bounce path with the robot starting at the start one and driving forwards.
public class H3_Bounce extends _DelayableStrafingAutoMode {
    Pose2d initialPose;

    @Override
    public Pose2d getInitialPose(){
        return initialPose;
    }
    
    public H3_Bounce(DriveSubsystem m_robotDrive) {
        String trajectoryJSON0 = "paths/output/Bounce00.wpilib.json";
        String trajectoryJSON1 = "paths/output/Bounce1.wpilib.json";
        String trajectoryJSON2 = "paths/output/Bounce2.wpilib.json";
        String trajectoryJSON3 = "paths/output/Bounce3.wpilib.json";

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

        Pose2d initialPoseTrajectory = trajectory0.getInitialPose();
        initialPose = new Pose2d(initialPoseTrajectory.getX(), initialPoseTrajectory.getY(), Rotation2d.fromDegrees(90));

        m_robotDrive.resetOdometry(initialPose); //because PathWeaver path uses absolute field coords
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(new WaitCommand(getInitialDelaySeconds()),
                createSwerveCommand(m_robotDrive, "Bounce: start to A3", 90, trajectory0),
                createSwerveCommand(m_robotDrive, "Bounce: A3 to A6", 90, trajectory1),
                createSwerveCommand(m_robotDrive, "Bounce: A6 to A9", 90, trajectory2),
                createSwerveCommand(m_robotDrive, "Bounce: A9 to end", 90, trajectory3));
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}

