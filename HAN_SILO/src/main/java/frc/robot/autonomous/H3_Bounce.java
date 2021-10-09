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
       String trajectoryJSON4 = "paths/output/Bounce4.wpilib.json";

        Trajectory trajectory0 = new Trajectory();
         Trajectory trajectory1 = new Trajectory();
         Trajectory trajectory2 = new Trajectory();
         Trajectory trajectory3 = new Trajectory();
        Trajectory trajectory4 = new Trajectory();
        try {
            Path traj0Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON0);
            trajectory0 = TrajectoryUtil.fromPathweaverJson(traj0Path);
            Path traj1Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
            trajectory1 = TrajectoryUtil.fromPathweaverJson(traj1Path);
            Path traj2Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
            trajectory2 = TrajectoryUtil.fromPathweaverJson(traj2Path);
            Path traj3Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON3);
            trajectory3 = TrajectoryUtil.fromPathweaverJson(traj3Path);
            Path traj4Path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON4);
            trajectory4 = TrajectoryUtil.fromPathweaverJson(traj4Path);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON0, ex.getStackTrace());
        }
        Pose2d initialPoseTrajectory = trajectory0.getInitialPose();
        initialPose = new Pose2d(initialPoseTrajectory.getX(), initialPoseTrajectory.getY(), Rotation2d.fromDegrees(90));

        m_robotDrive.resetOdometry(initialPose); //because PathWeaver path uses absolute field coords
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(new WaitCommand(getInitialDelaySeconds()),
                createSwerveCommand(m_robotDrive, "Bounce: entire path", 90, trajectory0),

                createSwerveCommand(m_robotDrive, "Bounce: start zone to A3", 90, trajectory1),
                 createSwerveCommand(m_robotDrive, "Bounce: A3 to A6", 90, trajectory2),
                 createSwerveCommand(m_robotDrive, "Bounce: A6 to A9", 90, trajectory3));


                // createSwerveCommand(m_robotDrive, "Bounce: start zone to A3",
                // TrajectoryDirection.REV,
                // TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                // {0.0, 0.0, -90}, //NOTE: robot starts with its +x (longitudinal) axis aligned
                // with field +x axis (facing the right side)
                // { 0, -10},
                // { 10, -20},
                // { 20, -30},
                // { 30, -35},
                // { 40, -40},
                // { 50, -45},
                // { 60, -50, 0} //A3 pylon
                // })


                // createSwerveCommand(m_robotDrive, "Bounce: A3 to A6", TrajectoryDirection.REV,
                //         TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { // these are INCHES
                //                 { 60, -50, 0 }, // A3 pylon

                //                 { -30, -80 }, { -60, -110 }, // low point
                //                 { -30, -140 },

                //                 { 30, -140 }, { 60, -140, 0 } // A6 pylon
                //         }),

                // createSwerveCommand(m_robotDrive, "Bounce: A6 to A9", TrajectoryDirection.REV,
                //         TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { // these are INCHES
                //                 { 60, -140, 0 }, // A6 pylon
                //                 { 30, -150 },

                //                 { -30, -160 }, { -55, -170 }, // low points
                //                 { -55, -185 }, { -55, -200 }, // low points
                //                 { -30, -210 },

                //                 { 30, -220 }, { 60, -230, 0 } // A9 pylon
                //         }),

                // createSwerveCommand(m_robotDrive, "Bounce: A9 to finish zone", TrajectoryDirection.REV,
                //         TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { // these are INCHES
                //                 { 60, -230, 0 }, // A9 pylon
                //                 { 30, -220 },

                //                 { 30, -240 }, { 10, -260 }, { 0, -280, 0 } // finish zone
                //         }));
        /*
         * createSwerveCommand(m_robotDrive, "Bounce", TrajectoryDirection.FWD,
         * TrajectoryHeading.UNROTATE, 0, new double[][] {{0.0, 0.0, -90}, {0.0, -0.5},
         * {0.0, -0.75}, {0.0, -1.0, -90}} ));
         */
        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }

}  

/*
H3_Bounce: Bounce: ﻿
﻿﻿﻿﻿﻿﻿ trajectory duration 2.0139250705796177 ﻿
﻿﻿﻿﻿﻿﻿ state 0                 poseMetersX 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 0                 poseMetersY 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 0         poseMetersTheta Deg 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 0     velocityMetersPerSecond 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 1                 poseMetersX -0.002136005002898488 ﻿
﻿﻿﻿﻿﻿﻿ state 1                 poseMetersY -0.2499894994694076 ﻿
﻿﻿﻿﻿﻿﻿ state 1         poseMetersTheta Deg 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 1     velocityMetersPerSecond 1.0 ﻿
﻿﻿﻿﻿﻿﻿ state 2                 poseMetersX 0.009079354468835014 ﻿
﻿﻿﻿﻿﻿﻿ state 2                 poseMetersY -0.7091892456306932 ﻿
﻿﻿﻿﻿﻿﻿ state 2         poseMetersTheta Deg 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 2     velocityMetersPerSecond 0.6387664429695296 ﻿
﻿﻿﻿﻿﻿﻿ state 3                 poseMetersX -0.04772132853083087 ﻿
﻿﻿﻿﻿﻿﻿ state 3                 poseMetersY -0.9695916629254524 ﻿
﻿﻿﻿﻿﻿﻿ state 3         poseMetersTheta Deg 0.0 ﻿
﻿﻿﻿﻿﻿﻿ state 3     velocityMetersPerSecond 0.15490503530257205 ﻿
﻿﻿﻿﻿﻿﻿ state 4                 poseMetersX -1.938496805434049E-4 ﻿
﻿﻿﻿﻿﻿﻿ state 4                 poseMetersY -0.9999952613225599 ﻿
﻿﻿﻿﻿﻿﻿ state 4         poseMetersTheta Deg -86.17518209721761 ﻿
﻿﻿﻿﻿﻿﻿ state 4     velocityMetersPerSecond 0.027850141159235203 ﻿
﻿﻿﻿﻿﻿﻿ state (end)             poseMetersX -1.3877787807814457E-17 ﻿
﻿﻿﻿﻿﻿﻿ state (end)             poseMetersY -1.0 ﻿
﻿﻿﻿﻿﻿﻿ state (end)     poseMetersTheta Deg -90.0 ﻿
﻿﻿﻿﻿﻿﻿ state (end) velocityMetersPerSecond 0.0 ﻿
﻿﻿﻿﻿﻿﻿ Running actual autonomous mode --> H3_Bounce ﻿

*///         {-1.524, -0.762},    
//         {-2.286, -1.524},    
//         {-5.334, -1.524},    
//         {-5.842, -0.762}, 
//         {-6.858, -0.0},
//         {-7.62, -0.762},
//         {-6.858, -1.524},
//         {-5.842, -0.762},
//         {-5.334, 0.0},
//         {-2.286, 0.0},
//         {-1.524, -0.762},
//         {-0.762, -1.524},