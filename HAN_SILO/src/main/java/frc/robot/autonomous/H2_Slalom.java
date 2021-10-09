package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.DriveSubsystem;

// This is the slalom path with the robot starting by the goal and driving backwards.
public class H2_Slalom extends _DelayableStrafingAutoMode {
    public H2_Slalom(DriveSubsystem m_robotDrive) {
                
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new WaitCommand(getInitialDelaySeconds()),

             createSwerveCommand(m_robotDrive, "Slalom: start zone to 1st cross", TrajectoryDirection.REV, 
                 TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                 {  0,    0, -90}, //NOTE: robot starts with its +x (longitudinal) axis aligned with field +x axis (facing the right side)
                 {  0,  -30},    
                 { 10,  -45},    
                 { 30,  -60, 0}   // 1st crisscross
             }),

             createSwerveCommand(m_robotDrive, "Slalom: 1st cross to 2nd cross", TrajectoryDirection.REV, 
                 TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                 { 30,  -60, 0}, //NOTE: robot starts with its +x (longitudinal) axis aligned with field +x axis (facing the right side)
                 { 50,  -75},    
                 { 60,  -150},    
                 { 50,  -225}, 
                 { 30,  -240, 0}   // 2nd crisscross
             }),
        
             createSwerveCommand(m_robotDrive, "Slalom: 2nd cross around 2nd cross", TrajectoryDirection.REV, 
                 TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                 { 30, -240, 0}, //NOTE: robot starts with its +x (longitudinal) axis aligned with field +x axis (facing the right side)
                 { 10, -255},    
                 { 10, -310},    
                 { 30, -315}, 
                 { 60, -310}, 
                 { 60, -225}, 
                 { 30, -240, 0}   // 2nd crisscross around to 2nd crisscross
             }),
        
             createSwerveCommand(m_robotDrive, "Slalom: 2nd cross back to 1st cross", TrajectoryDirection.REV, 
                 TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                 { 30, -240, 0}, //NOTE: robot starts with its +x (longitudinal) axis aligned with field +x axis (facing the right side)
                 { 10, -225},    
                 {  0, -150},    
                 { 10,  -75}, 
                 { 30,  -60, 0}   // 2nd crisscross to 1st crisscross
             }),
        
             createSwerveCommand(m_robotDrive, "Slalom: 1st cross back to finish", TrajectoryDirection.REV, 
                 TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][] { //these are INCHES
                 { 30, -60, 0},
                 { 50, -45},
                 { 60, -30}, 
                 { 60,   0, 0}  // finish zone
             })
         );

         // Run path following command, then stop at the end.
        command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}  

// public class H2_Slalom extends _DelayableStrafingAutoMode {
//     public H2_Slalom(DriveSubsystem m_robotDrive) {
                
//         SequentialCommandGroup commandGroup = new SequentialCommandGroup(
//             new WaitCommand(getInitialDelaySeconds()),
//             createSwerveCommand(m_robotDrive, "Slalom ", TrajectoryDirection.REV, 
//             TrajectoryHeading.CONVERT_TO_METERS, 0, new double[][]
//                 {{0, 0, -90},
//                 {-30, -0.0},    
//                 {-60, -30},    
//                 {-90, -60},    
//                 {-210, -60},    
//                 {-230, -30}, 
//                 {-270, -0.0},
//                 {-300, -30},
//                 {-270, -60},
//                 {-230, -30},
//                 {-210, 0.0},
//                 {-90, 0.0},
//                 {-60, -30},
//                 {-30, -60},
//                 {0.0, -60,0}}

//             ));

//         // Run path following command, then stop at the end.
//         command = commandGroup.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
//     }
// }
