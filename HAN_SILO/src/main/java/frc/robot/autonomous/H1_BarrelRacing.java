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
public class H1_BarrelRacing extends _DelayableStrafingAutoMode {
    Pose2d initialPose;

    @Override
    public Pose2d getInitialPose(){
        return initialPose;
    }
    
    public H1_BarrelRacing(DriveSubsystem m_robotDrive) {
        String trajectoryJSON0 = "paths/output/BarrelRacing0.wpilib.json";
        //String trajectoryJSON1 = "paths/output/Bounce1.wpilib.json";
        //String trajectoryJSON2 = "paths/output/Bounce2.wpilib.json";
        //String trajectoryJSON3 = "paths/output/Bounce3.wpilib.json";
     //   String trajectoryJSON4 = "paths/output/Bounce4.wpilib.json";

        Trajectory trajectory0 = new Trajectory();
        // Trajectory trajectory1 = new Trajectory();
         //Trajectory trajectory2 = new Trajectory();
         //Trajectory trajectory3 = new Trajectory();
        // Trajectory trajectory4 = new Trajectory();
        try {
            Path traj0Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON0);
            trajectory0 = TrajectoryUtil.fromPathweaverJson(traj0Path);
            //Path traj1Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
            //trajectory1 = TrajectoryUtil.fromPathweaverJson(traj1Path);
            //Path traj2Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
            //trajectory2 = TrajectoryUtil.fromPathweaverJson(traj2Path);
            //Path traj3Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON3);
           // trajectory3 = TrajectoryUtil.fromPathweaverJson(traj3Path);
         //   Path traj4Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON4);
         //    trajectory4 = TrajectoryUtil.fromPathweaverJson(traj4Path);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON0, ex.getStackTrace());
        }
        Pose2d initialPoseTrajectory = trajectory0.getInitialPose();
        initialPose = new Pose2d(initialPoseTrajectory.getX(), initialPoseTrajectory.getY(), Rotation2d.fromDegrees(90));

        m_robotDrive.resetOdometry(initialPose); //because PathWeaver path uses absolute field coords
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(new WaitCommand(getInitialDelaySeconds()),
                createSwerveCommand(m_robotDrive, "Bounce: entire path", 90, trajectory0));


        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }

}  

