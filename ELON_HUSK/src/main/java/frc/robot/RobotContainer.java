/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.F1_Move_Forward;
import frc.robot.autonomous._NamedAutoMode;
import frc.robot.autonomous._NotImplementedProperlyException;
import frc.robot.commands.ResetEncodersCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.climb.ClimbDownCommand;
import frc.robot.commands.climb.ClimbUpCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.intake.LeftIntakeCommand;
import frc.robot.commands.intake.RightIntakeCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	XboxController m_driverController = new XboxController(Constants.kDriverControllerPort);
	Joystick m_operatorController = new Joystick(Constants.kOperatorControllerPort);

	private DriveSubsystem m_drive;
	private IntakeSubsystem m_intake;
	private LimeLightSubsystem m_vision;
	private ClimbSubsystem m_climb;

	public enum HanMode {
		MODE_SHOOT, MODE_CLIMB
	}

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 * 
	 * @throws _NotImplementedProperlyException
	 */
	public RobotContainer(ClimbSubsystem climb, DriveSubsystem drive, IntakeSubsystem intake, LimeLightSubsystem vision) {
		// this.m_ledstring = m_ledstring;
		this.m_drive = drive;
		this.m_intake = intake;
		this.m_vision = vision;
		this.m_climb = climb;

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		// Set the default drive command to split-stick arcade drive
		m_drive.setDefaultCommand(
				// A split-stick arcade command, with forward/backward controlled by the left
				// hand, and turning controlled by the right.
				new RunCommand(() -> m_drive.drive(
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

						m_drive));
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * calling passing it to a {@link JoystickButton}.
	 */
	private void configureButtonBindings() {

		//#region Drive Subsystem
		new JoystickButton(m_driverController, ButtonConstants.kResetGyro).whenPressed(new ResetGyroCommand(m_drive));
		new JoystickButton(m_driverController, ButtonConstants.kResetEncoders).whenPressed(new ResetEncodersCommand(m_drive));
		//#endregion
		
		//#region Climb Subsystem
		new JoystickButton(m_operatorController, ButtonConstants.kClimbUp).whenHeld(new ClimbUpCommand(m_climb));
		new JoystickButton(m_operatorController, ButtonConstants.kClimbDown).whenHeld(new ClimbDownCommand(m_climb));
		//#endregion
		
		//left = button 1
		//right = button 12
		new JoystickButton(m_operatorController, 1).whenHeld(new LeftIntakeCommand(m_intake));
		new JoystickButton(m_operatorController, 12).whenHeld(new RightIntakeCommand(m_intake));
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
				selectedAutoMode = new _NamedAutoMode(new F1_Move_Forward(m_drive));
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
				return new _NamedAutoMode(new F1_Move_Forward(m_drive));

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
