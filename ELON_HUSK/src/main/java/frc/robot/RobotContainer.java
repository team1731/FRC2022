/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.F1_Move_Forward;
import frc.robot.autonomous.T1_Move_Test;
import frc.robot.autonomous._NamedAutoMode;
import frc.robot.autonomous._NotImplementedProperlyException;
import frc.robot.commands.VisionRotateCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.Trigger;
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
	private IntakeSubsystem m_intake;
	private LimeLightSubsystem m_vision;

	public enum HanMode {
		MODE_SHOOT, MODE_CLIMB
	}

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 * 
	 * @throws _NotImplementedProperlyException
	 */
	public RobotContainer(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake, LimeLightSubsystem m_vision) {
		// this.m_ledstring = m_ledstring;
		this.m_robotDrive = m_robotDrive;
		this.m_intake = m_intake;
		this.m_vision = m_vision;

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

		// Activate Intake via Operator Left Axis/Trigger
		// new HanTrigger(HanTriggers.DR_TRIG_RIGHT).whileActiveContinuous(new ShootSeqCommand(m_shootclimb, m_sequencer),
		// 		true);

		// Map right bumper to rotation lock to power port
		new JoystickButton(m_driverController, XboxConstants.kRBumper)
				.whenActive(new VisionRotateCommand(m_vision, m_robotDrive, m_driverController));

		new JoystickButton(m_driverController, 7).whenPressed(new InstantCommand(() -> {
			m_robotDrive.resetGyro();
			System.out.println("Reset gyro");
		}, m_robotDrive));
		new JoystickButton(m_driverController, XboxConstants.kMenu).whenPressed(new InstantCommand(() -> {
			m_robotDrive.resetEncoders();
			System.out.println("Reset encoders");
		}, m_robotDrive));

	}

	public _NamedAutoMode getNamedAutonomousCommand(String autoSelected) {
		String autoMode = "";
		int initialDelaySeconds = 0;
		int secondaryDelaySeconds = 0;
		if (autoSelected.length() > 1) {
			autoMode = autoSelected.substring(0, 2);
		}
		if (autoSelected.length() > 2) {
			try {
				initialDelaySeconds = Integer.parseInt(autoSelected.substring(2, 2));
			} catch (Exception e) {
				System.out.println("INITIAL DELAY did not parse -- defaulting to 0 seconds!!!");
			}
		}
		if (autoSelected.length() > 3) {
			try {
				secondaryDelaySeconds = Integer.parseInt(autoSelected.substring(3, 3));
			} catch (Exception e) {
				System.out.println("SECONDARY DELAY did not parse -- defaulting to 0 seconds!!!");
			}
		}

		_NamedAutoMode selectedAutoMode = null;

		try {
			selectedAutoMode = createNamedAutoMode(autoMode);
		} catch (_NotImplementedProperlyException e) {
			System.err.println("SELECTED MODE NOT IMPLEMENTED -- DEFAULT TO F1_MOVE_FORWARD!!!");
			try {
				selectedAutoMode = new _NamedAutoMode(new F1_Move_Forward(m_robotDrive));
			} catch (_NotImplementedProperlyException e2) {
				System.err.println("F1_Move_Forward could NOT be created -- Aborting!!!");
				return null;
			}
		}
		if (selectedAutoMode != null) {
			selectedAutoMode.delayableStrafingAutoMode.setInitialDelaySeconds(initialDelaySeconds);
			selectedAutoMode.delayableStrafingAutoMode.setSecondaryDelaySeconds(secondaryDelaySeconds);
		}

		return selectedAutoMode;
	}

	private _NamedAutoMode createNamedAutoMode(String autoModeName) throws _NotImplementedProperlyException {
		switch (autoModeName) {
			case "F1":
				return new _NamedAutoMode(new F1_Move_Forward(m_robotDrive));
			case "T1":
				return new _NamedAutoMode(new T1_Move_Test(m_robotDrive));
			default:
				System.err.println("FATAL: SELECTED AUTO MODE " + autoModeName + " DOES NOT MAP TO A JAVA CLASS!!!!");
				return null;
		}
	}

	// Enables Use of controller axis/trigger by creating a Custom Trigger
	

	// Controller Triggers
	public enum HanTriggers {
		DR_TRIG_LEFT, DR_TRIG_RIGHT, OP_TRIG_LEFT, OP_TRIG_RIGHT
	}

	public class HanTrigger extends Trigger {
		HanTriggers desired;
		double triggerValue = 0;
	
		public HanTrigger(HanTriggers selected) {
			this.desired = selected;
		}
	
		@Override
		public boolean get() {
			switch (desired) {
				case DR_TRIG_LEFT:
					triggerValue = m_driverController.getLeftTriggerAxis();
					break;
				case DR_TRIG_RIGHT:
					triggerValue = m_driverController.getRightTriggerAxis();
					break;
				case OP_TRIG_LEFT:
					// triggerValue = m_operatorController.getLeftTriggerAxis();
					break;
				case OP_TRIG_RIGHT:
					// triggerValue = m_operatorController.getRightTriggerAxis();
					break;
			}
			return (Math.abs(triggerValue) > 0.5);
		}
	}
}
