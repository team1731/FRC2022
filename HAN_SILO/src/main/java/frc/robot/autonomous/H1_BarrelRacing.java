package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.DriveSubsystem;

public class H1_BarrelRacing extends _DelayableStrafingAutoMode {
    public H1_BarrelRacing(DriveSubsystem m_robotDrive) {
                
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),

             createSwerveCommand(m_robotDrive, "BarrelRacing: start zone to around 1st circle", TrajectoryDirection.REV, 
                 TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                 {   0,    0, -90},
                 {   0,  -30},
                 {   0, -120},    
                 { -30, -150},    
                 { -60, -120},    
                 { -30,  -90}, 
                 {   0, -120,  0}   // 1st circle
             }),

             createSwerveCommand(m_robotDrive, "BarrelRacing: around 2nd circle", TrajectoryDirection.REV, 
                 TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                 {   0, -120,  0},  // 1st circle
                 {  15, -240}, 
                 {  45, -240}, 
                 {  45, -180}, 
                 {  15, -180, 0}   // 2nd circle
             }),
        
             createSwerveCommand(m_robotDrive, "BarrelRacing: around 3rd circle", TrajectoryDirection.REV, 
                 TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                 {  15, -180,  0},  // 2nd circle
                 { -60, -240}, 
                 { -60, -300}, 
                 { -10, -300, 0}   // 3rd circle
             }),
                
             createSwerveCommand(m_robotDrive, "BarrelRacing: back to finish", TrajectoryDirection.REV, 
                 TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                 { -10, -300, 0}, // 3rd circle
                 {  -5, -180},
                 {   0,    0, 0}  //finish zone
             })
         );

         // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}

