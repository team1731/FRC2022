package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeRetract;
import frc.robot.commands.IntakeSeqCommand;
import frc.robot.commands.ShootSeqCommandAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;

// This is the bounce path with the robot starting at the start one and driving forwards.
public class H0_GalacticSearch extends _DelayableStrafingAutoMode {
    Pose2d initialPose;
    Integer field_orientation;

    @Override
    public Pose2d getInitialPose(){
        return initialPose;
    }

    @Override
    public Integer getFieldOrientation(){
        return field_orientation;
    }

    public H0_GalacticSearch(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, SequencerSubsystem m_sequence,
                             LimeLightSubsystem m_limelight, ShootClimbSubsystem m_shootclimb) {

        SequentialCommandGroup commandGroup = null;
        Path trajectoryPath = null;
        String trajectoryName = "";

        field_orientation = m_limelight.getFieldOrientation(); 
        switch(field_orientation){
            case 0: trajectoryName =  "RedPathA"; break; // Red A (C3, D5, A6)
            case 1: trajectoryName =  "RedPathB"; break; // Red B (B3, D5, B7)
            case 2: trajectoryName = "BluePathA"; break; //Blue A (E6, B7, C9)
            case 3: trajectoryName = "BluePathB"; break; //Blue B (D6, B8, D10)
        }       
        try {
            SmartDashboard.putString("SelectedGalactic", trajectoryName);
            System.out.println("\nConstructing " + trajectoryName + " auto\n");
            trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/output/" + trajectoryName + ".wpilib.json");
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

            Pose2d initialPoseTrajectory = trajectory.getInitialPose();
            initialPose = new Pose2d(initialPoseTrajectory.getX(), initialPoseTrajectory.getY(), Rotation2d.fromDegrees(180.0));

            m_robotDrive.resetOdometry(initialPose); //because PathWeaver path uses absolute field coords
            commandGroup = new SequentialCommandGroup(new WaitCommand(getInitialDelaySeconds()),
                new ParallelCommandGroup(
                    new IntakeSeqCommand(m_intake, m_sequence, true).withTimeout(6),
                    createSwerveCommand(m_robotDrive, trajectoryName, 180.0, trajectory)
                ),
                new IntakeRetract(m_intake),
                new ShootSeqCommandAuto(m_shootclimb, m_sequence).withTimeout(5)
            );
        } catch (IOException ex){
            System.out.println("Path not found: " + trajectoryPath);
            DriverStation.reportError("Unable to open trajectory " + trajectoryName, ex.getStackTrace());
            commandGroup = new SequentialCommandGroup(new WaitCommand(getInitialDelaySeconds()));
        } finally {
            System.out.println("Path: " + trajectoryPath);
        }
        
        // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false)).andThen(() -> m_shootclimb.stopShooting());
    }
}  
