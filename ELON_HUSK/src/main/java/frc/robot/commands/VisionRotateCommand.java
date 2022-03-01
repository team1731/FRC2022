package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Rotates the robot towards a vision target
 */
public class VisionRotateCommand extends CommandBase {

	private final VisionSubsystem m_vision;
	private final DriveSubsystem m_drive;

	public VisionRotateCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
		m_vision = visionSubsystem;
		m_drive = driveSubsystem;
		addRequirements(visionSubsystem, driveSubsystem);
	}

	@Override
	public void initialize() {
		// Tell the DriveSubsystem to take turning control away from driver
		m_drive.setVisionHeadingOverride(true);
	}

	@Override
	public void execute() {
		// Update the turning PID goal if a valid target exists
		if (m_vision.hasTarget()) {
			double targetAngle = m_drive.getHeading() - m_vision.getLastTarget().getY();
			SmartDashboard.putNumber("Vis_TargetAngle", targetAngle);
			m_drive.setVisionHeadingGoal(targetAngle);
		}
	}

	@Override
	public void end(boolean interrupted) {
		// Give control back to driver
		m_drive.setVisionHeadingOverride(false);
	}

}