/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.C2_B2X2;
import frc.robot.autonomous.C4_B2X2_B4B5X2;
import frc.robot.autonomous.F1_Move_Forward;
import frc.robot.autonomous.L2_B3X2;
import frc.robot.autonomous.L3_B3X2_B5X2;
import frc.robot.autonomous.L4_B3L2_B5B4L2;
import frc.robot.autonomous.L5_B3X2_B5B4X2_B2B1X1;
import frc.robot.autonomous.L6_B3X2_B5B4X2_B2X1;
import frc.robot.autonomous.R2_B1X2;
import frc.robot.autonomous.R5_X2B2X1B4B5X2;
import frc.robot.autonomous.X0_DoNothing;
import frc.robot.autonomous._NamedAutoMode;
import frc.robot.autonomous._NotImplementedProperlyException;
import frc.robot.commands.ResetEncodersCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.intake.RightIntakeJoyconCommand;
import frc.robot.commands.intake.LeftIntakeJoyconCommand;
import frc.robot.commands.climb.ClimbDownCommand;
import frc.robot.commands.climb.ClimbUpCommand;
import frc.robot.commands.climb.RewindClimbComand;
import frc.robot.commands.intake.RightIntakeEjectCommand;
import frc.robot.commands.intake.LeftIntakeEjectCommand;
import frc.robot.commands.launch.LaunchBallCommand;
import frc.robot.commands.launch.LaunchManualEnableCommand;
import frc.robot.commands.launch.LaunchManualDisableCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.DriveConstants;

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
	private LaunchSubsystem m_launch;
	private IntakeSubsystem m_intake;
	private VisionSubsystem m_vision;
	private ClimbSubsystem m_climb;

	public enum HanMode {
		MODE_SHOOT, MODE_CLIMB
	}

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 * 
	 * @throws _NotImplementedProperlyException
	 */
	public RobotContainer(
			DriveSubsystem drive, 
			ClimbSubsystem climb,
			LaunchSubsystem launch, 
			IntakeSubsystem intake, 
			VisionSubsystem vision)
		{
		// this.m_ledstring = m_ledstring;
		this.m_drive = drive;
		this.m_launch = launch;
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

						-m_driverController.getRightY() * Math.abs(m_driverController.getRightY()), true,

						m_driverController.getRightBumper()),

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
		new JoystickButton(m_operatorController, ButtonConstants.kClimbUp).whileHeld(new ClimbUpCommand(m_climb));
		new JoystickButton(m_operatorController, ButtonConstants.kClimbDown).and(new JoystickButton(m_operatorController, ButtonConstants.kClimbSensorOverride)).whileActiveContinuous(new ClimbDownCommand(m_climb));
	


	//	new JoystickButton(m_operatorController, ButtonConstants.kClimbSensorOverride).whileHeld(new OverrideSensorCommand(m_climb));
		//#endregion

		//#region Vision Subsystem
	//	new JoystickButton(m_driverController, ButtonConstants.kVision).whenHeld(new VisionRotateCommand(m_vision, m_drive));
		//#endregion
		
		//#region Intake Subsystem
		//Alternate code for Joystick testing as opposed to simulation
		new JoystickButton(m_operatorController, ButtonConstants.kIntakeLeft).whenHeld(new LeftIntakeJoyconCommand(m_intake));
		new JoystickButton(m_operatorController, ButtonConstants.kIntakeRight).whenHeld(new RightIntakeJoyconCommand(m_intake));
		new HanTrigger(HanTriggers.DR_TRIG_LEFT).whileActiveContinuous(new LeftIntakeJoyconCommand(m_intake));
		new HanTrigger(HanTriggers.DR_TRIG_RIGHT).whileActiveContinuous(new RightIntakeJoyconCommand(m_intake));	 
		new JoystickButton(m_operatorController, ButtonConstants.kIntakeLeftEject).whenHeld(new LeftIntakeEjectCommand(m_intake));
		new JoystickButton(m_operatorController, ButtonConstants.kIntakeRightEject).whenHeld(new RightIntakeEjectCommand(m_intake));
		new JoystickButton(m_operatorController, ButtonConstants.kRewindClimber).whenHeld(new RewindClimbComand(m_climb));

		// //left = button 1
		// //right = button 12
		// new JoystickButton(m_operatorController, ButtonConstants.kIntakeLeft).whenActive(new LeftIntakeCommand(m_intake)).whenInactive(new LeftStopCommand(m_intake));
		// new JoystickButton(m_operatorController, ButtonConstants.kIntakeRight).whenActive(new RightIntakeCommand(m_intake)).whenInactive(new RightStopCommand(m_intake));
		//#endregion

		//#region Launch Subsystem



		new JoystickButton(m_driverController, ButtonConstants.kLaunchBall).whenHeld(new LaunchBallCommand(m_launch));
		new JoystickButton(m_operatorController, ButtonConstants.kRobotModeShoot)
			.whileActiveContinuous(() -> m_launch.runLaunch(
					(m_operatorController.getRawAxis(4)+1)/2,
					(m_operatorController.getRawAxis(1)+1)/2,m_driverController.getRightBumper() 
				)
			)
			.whenInactive(() -> m_launch.stopLaunch());




			
		new JoystickButton(m_operatorController, ButtonConstants.kLaunchManualMode)
			.whenActive(new LaunchManualEnableCommand(m_launch))
			.whenInactive(new LaunchManualDisableCommand(m_launch));
		//#endregion
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
			case "L2":
			    return new _NamedAutoMode(new L2_B3X2(m_drive, m_intake, m_launch));
			case "L3":
			    return new _NamedAutoMode(new L3_B3X2_B5X2(m_drive, m_intake, m_launch));
			case "L4":
			    return new _NamedAutoMode(new L4_B3L2_B5B4L2(m_drive, m_intake, m_launch));
			case "L5":
			    return new _NamedAutoMode(new L5_B3X2_B5B4X2_B2B1X1(m_drive, m_intake, m_launch));
			case "L6":
			    return new _NamedAutoMode(new L6_B3X2_B5B4X2_B2X1(m_drive, m_intake, m_launch));
			case "R2":
			    return new _NamedAutoMode(new R2_B1X2(m_drive, m_intake, m_launch));
			case "R5":
			    return new _NamedAutoMode(new R5_X2B2X1B4B5X2(m_drive, m_intake, m_launch));
			case "C2":
			    return new _NamedAutoMode(new C2_B2X2(m_drive, m_intake, m_launch));
			case "X0":
				return new _NamedAutoMode(new X0_DoNothing(m_drive, m_intake, m_launch));
		
			case "C4":
			default:
				return new _NamedAutoMode(new C4_B2X2_B4B5X2(m_drive, m_intake, m_launch));
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
Button(1) = top Left back toggle: Left Intake (Towards Driver = ON);
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
Button(12) = top Right back toggle: Right Intake (Toggle towards Driver)
Button(14) = Front Left Bottom(1): Eject (Top Button)
Button(15) = front Left Bottom(2): Pickup (Bottom Button)
Button(16) = front Right Bottom: Unspecified (Clicking the scrollwheel thing)
*/