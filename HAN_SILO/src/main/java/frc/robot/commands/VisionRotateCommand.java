package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.XboxConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.DetectionMode;

/**
 * Rotates the robot towards a vision target
 */
public class VisionRotateCommand extends CommandBase {

    private final LimeLightSubsystem m_vision;
    private final DriveSubsystem m_drive;
    private final XboxController m_driverController;

    public VisionRotateCommand(LimeLightSubsystem visionSubsystem, DriveSubsystem driveSubsystem, XboxController driveController){
        m_vision = visionSubsystem;
        m_drive = driveSubsystem;
        m_driverController = driveController;
    }

    @Override
    public void initialize(){
        //Turn on LED and tell the DriveSubsystem to take turning control away from driver
        m_drive.setVisionHeadingOverride(true);
        m_vision.enableLED();
        m_vision.SetDetectionMode(DetectionMode.PowerPort);
    }

    @Override
    public void execute(){
        //Update the turning PID goal if a valid target exists
        if(m_vision.hasTarget()){
            double targetAngle = m_drive.getHeading() - m_vision.getLastPortPos().getY();
            SmartDashboard.putNumber("Vis_TargetAngle", targetAngle);
            m_drive.setVisionHeadingGoal(targetAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
        //Turn off LED and give control back to driver
        m_drive.setVisionHeadingOverride(false);
        m_vision.disableLED();
    }

    @Override
    public boolean isFinished(){
        //When the right bumper is released, stop the command
        return !m_driverController.getRawButton(XboxConstants.kRBumper);
    } 

}