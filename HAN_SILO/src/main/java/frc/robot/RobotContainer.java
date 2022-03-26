/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.XboxConstants;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

	private DriveSubsystem m_robotDrive;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer(DriveSubsystem m_robotDrive) {
		// this.m_ledstring = m_ledstring;
		this.m_robotDrive = m_robotDrive;

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		// Set the default drive command to split-stick arcade drive
		m_robotDrive.setDefaultCommand(
				// A split-stick arcade command, with forward/backward controlled by the left
				// hand, and turning controlled by the right.
				new RunCommand(() -> m_robotDrive.drive(
						// Get the x speed. We are inverting this because Xbox controllers return
						// negative values when we push forward.
						-m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY())
								* DriveConstants.kMaxSpeedMetersPerSecond,

						// Get the y speed or sideways/strafe speed. We are inverting this because
						// we want a positive value when we pull to the left. Xbox controllers
						// return positive values when you pull to the right by default.
						-m_driverController.getLeftX() * Math.abs(m_driverController.getLeftX())
								* DriveConstants.kMaxSpeedMetersPerSecond,

						// Get the rate of angular rotation. We are inverting this because we want a
						// positive value when we pull to the left (remember, CCW is positive in
						// mathematics). Xbox controllers return positive values when you pull to
						// the right by default.
						-m_driverController.getRightX() * Math.abs(m_driverController.getRightX()),

						-m_driverController.getRightY() * Math.abs(m_driverController.getRightY()),

						true),

						m_robotDrive));
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * calling passing it to a {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
		// Reset Gyro
		new JoystickButton(m_driverController, 7).whenPressed(new InstantCommand(() -> {
			m_robotDrive.resetGyro();
		}, m_robotDrive));

		// Reset Encoders
		new JoystickButton(m_driverController, XboxConstants.kMenu).whenPressed(new InstantCommand(() -> {
			m_robotDrive.resetEncoders();
		}, m_robotDrive));
	}
}
