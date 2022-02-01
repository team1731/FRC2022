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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VelocityTalonFX;
import frc.robot.subsystems.VelocityNeo;
import frc.robot.subsystems.SmartMotionNeo;

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

	// The robot's subsystems
	private IntakeSubsystem m_intake;
	private VelocityTalonFX m_talon;
	private VelocityNeo m_neo;
	private SmartMotionNeo m_smart;

	public enum HanMode {
		MODE_SHOOT, MODE_CLIMB
	}

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 * 
	 * @throws _NotImplementedProperlyException
	 */
	public RobotContainer(IntakeSubsystem intake, VelocityTalonFX talon, VelocityNeo neo, SmartMotionNeo smart) {
		// this.m_ledstring = m_ledstring;
		this.m_intake = intake;
		this.m_talon = talon;
		this.m_neo = neo;
		this.m_smart = smart;

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		// Set the default drive command to split-stick arcade drive
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * calling passing it to a {@link JoystickButton}.
	 */
	private void configureButtonBindings() {

		//#region Drive Subsystem
		//#endregion
		
		//#region Test Subsystem
		new JoystickButton(m_operatorController, 8) // convert -1 to +1 TO 0 to 1
	//		.whileActiveContinuous(() -> m_launcher.spinLauncher((m_operatorController.getRawAxis(1)+1)/2))
			.whileActiveContinuous(() -> m_neo.spinNeo(m_operatorController.getRawAxis(1)))
			.whenInactive(() -> m_neo.stopLaunching());

		new JoystickButton(m_operatorController, 9)
			.whileActiveContinuous(() -> m_talon.spinIntake((m_operatorController.getRawAxis(4)+1)/2))
			.whenInactive(() -> m_talon.stopIntake());

		new JoystickButton(m_operatorController, 7)
			.whileActiveContinuous(() -> m_smart.spinSmart(m_operatorController.getRawAxis(1)))
			.whenInactive(() -> m_smart.stopSmart());
		//#endregion
		
		//left = button 1
		//right = button 12
		new JoystickButton(m_operatorController, 1).whenHeld(new LeftIntakeCommand(m_intake));
		new JoystickButton(m_operatorController, 12).whenHeld(new RightIntakeCommand(m_intake));

		//#
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

//Driver Controls List

//Operator controls List
/*
//AXES = JOYSTICKS/BACK WHEEL CONTROLS
getRawAxis(0) = left joystick(L/R): Unspecified
getRawAxis(1) = left joystick(U/D): Extend/Retract Intake
getRawAxis(2) = left back wheel: Unspecified
getRawAxis(3) = right joystick(L/R): Unspecified
getRawAxis(4) = right joystick(U/D): shooter speed
getRawAxis(5) = right back wheel: Unspecified

//BUTTONS
Button(1) = top Left back toggle: Unspecified (Towards Driver = ON);
Button(2) = top Left front toggle: Unspecified (Toggle away from Driver)
Button(3) = top Left front toggle: Unspecified (Toggle towards Driver)
Button(4) = front Left front toggle(L): Unspecified (Toggle away from Driver)
Button(5) = front Left front toggle(L): Unspecified (Toggle towards Driver)
Button(6) = front Left front toggle(R): Climb Up (Toggle away from Driver)
Button(7) = front Left front toggle(R): Climb Down (Toggle towards Driver)
Button(8) = front Right front toggle(R): Shoot (Toggle towards Driver)
Button(9) = front Right front toggle(R): Climb (Toggle away from Driver)
Button(10) = top Right front toggle: Unspecified (Toggle towards Driver)
Button(11) = top Right front toggle: Unspecified (Toggle away from Driver)
Button(12) = top Right back toggle: Unspecified (Toggle towards Driver)
Button(14) = Front Left Bottom(1): Eject (Top Button)
Button(15) = front Left Bottom(2): Pickup (Bottom Button)
Button(16) = front Right Bottom: Unspecified (Clicking the scrollwheel thing)
*/