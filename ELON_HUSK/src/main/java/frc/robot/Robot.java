/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.autonomous._NamedAutoMode;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	private _NamedAutoMode namedAutoMode;
	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;

	// The robot's modules ** DOES NOT WORK with CAN Id of 21, needs to be 1 but drive motor is 1

	// The robot's subsystems
	public DriveSubsystem m_drive;
	public VisionSubsystem m_vision;
	public LaunchSubsystem m_launch;
	public IntakeSubsystem m_intake;
	public ClimbSubsystem m_climb;

	String autoCode = AutoConstants.kDEFAULT_AUTO_CODE;

	public Robot() {
		addPeriodic(() -> {
			if (m_drive != null)
				m_drive.updateOdometry();
		}, 0.010, 0.0); // was 0.005
	}

	private void initSubsystems() {
		// initial SubSystems to at rest states
		m_drive.resetEncoders();
		if (m_intake != null) {
			m_intake.retract();
		}

	}

	private void autoInitPreload() {
		System.out.println("autoInitPreload: Start");
		m_autonomousCommand = null;
		m_drive.resetOdometry(new Pose2d());

		m_drive.resumeCSVWriter();

		if (RobotBase.isReal()) {
			autoCode = SmartDashboard.getString(AutoConstants.kAUTO_CODE, autoCode);
		}
		System.out.println("AUTO CODE retrieved from Dashboard --> " + autoCode);
		if (autoCode == null || autoCode.length() < 2) {
			autoCode = AutoConstants.kDEFAULT_AUTO_CODE;
			SmartDashboard.putString(AutoConstants.kAUTO_CODE, AutoConstants.kDEFAULT_AUTO_CODE);
		}
		autoCode = autoCode.toUpperCase();
		System.out.println("AUTO CODE being used by the software --> " + autoCode);

		m_autonomousCommand = null;
		namedAutoMode = m_robotContainer.getNamedAutonomousCommand(autoCode);
		if (namedAutoMode != null) {
			System.out.println("autoInitPreload: getCommand Auto Begin: "+namedAutoMode.name);
			m_autonomousCommand = namedAutoMode.getCommand();
			System.out.println("autoInitPreload: getCommand Auto Complete");
		} else {
			System.err.println("UNABLE TO EXECUTE SELECTED AUTONOMOUS MODE!!");
		}
		System.out.println("autoInitPreload: End");
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
        LiveWindow.disableAllTelemetry();
		// CameraServer camServer = CameraServer.getInstance();
		// camServer.startAutomaticCapture();

		m_vision = new VisionSubsystem();
		m_drive = new DriveSubsystem(m_vision);
		m_launch = new LaunchSubsystem(m_drive);
		m_intake = new IntakeSubsystem();
		m_climb = new ClimbSubsystem();

		//m_pdp.clearStickyFaults();
		//m_pneu.clearAllStickyFaults();
		m_drive.zeroHeading();

		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer(m_drive, m_climb, m_launch, m_intake, m_vision);

		initSubsystems();
		if (m_launch != null) {
			m_launch.resetEncoder();
		}

		SmartDashboard.putString(AutoConstants.kAUTO_CODE, AutoConstants.kDEFAULT_AUTO_CODE); // see above for explanation
		if (RobotBase.isReal()) {
			autoCode = SmartDashboard.getString(AutoConstants.kAUTO_CODE, autoCode);
		}
        
		autoInitPreload();
		//m_climb.handleReady();

	//	SmartDashboard.putBoolean("Vis_HasTarget", false);
	//	SmartDashboard.putNumber("Vis_TargetAngle", 0);

		SmartDashboard.putString("Build Info - Branch", "N/A");
		SmartDashboard.putString("Build Info - Commit Hash", "N/A");
		SmartDashboard.putString("Build Info - Date", "N/A");

		try {
			File buildInfoFile = new File(Filesystem.getDeployDirectory(), "DeployedBranchInfo.txt");
			Scanner reader = new Scanner(buildInfoFile);
			int i = 0;
			while(reader.hasNext()){
				if(i == 0){
					SmartDashboard.putString("Build Info - Branch", reader.nextLine());
				} else if(i == 1){
					SmartDashboard.putString("Build Info - Commit Hash", reader.nextLine());
				} else {
					SmartDashboard.putString("Build Info - Date", reader.nextLine());
				}
				i++;
			}
			
			reader.close();
		} catch (FileNotFoundException fnf) {
			System.err.println("DeployedBranchInfo.txt not found");
			fnf.printStackTrace();
		}
    
		
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();

		

	//	SmartDashboard.putNumber("PCM Current", m_pneu.getCompressorCurrent());
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
		m_drive.suspendCSVWriter();
		m_vision.disableLED(false);
	}

	@Override
	public void disabledPeriodic() {
		/*
		 * if(m_ledstring != null){ m_ledstring.option(LedOption.TEAM); }
		 */
		m_drive.resetEncoders();
		m_climb.doSD();
		m_drive.doSD();
		if (System.currentTimeMillis() % 100 == 0) {
			// SmartDashboard.putBoolean("LowSensor", m_sequencer.lowSensorHasBall());
			// SmartDashboard.putBoolean("MidSensor", m_sequencer.midSensorHasBall());
			// SmartDashboard.putBoolean("HighSensor", m_sequencer.highSensorHasBall());
		}

		 if (RobotBase.isReal()) {
		 	String newCode = SmartDashboard.getString(AutoConstants.kAUTO_CODE, autoCode);
		 	if (!newCode.equals(autoCode)) {
		 		System.out.println("New Auto Code read from dashboard. OLD: \""+autoCode+"\", NEW: \""+newCode+"\" - initializing.");
		 		autoCode = newCode;
		 		autoInitPreload();
		 	}
		 }
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		m_drive.resumeCSVWriter();
		if (m_launch != null) {
			m_launch.resetEncoder();
		}
		CommandScheduler.getInstance().cancelAll();
      //  m_drive.setCurrentLimits(false);
		// schedule the autonomous command (example)
		if (m_autonomousCommand == null) {
			System.err.println("SOMETHING WENT WRONG - UNABLE TO RUN AUTONOMOUS! CHECK SOFTWARE!");
		} else {
			System.out.println("Running actual autonomous mode --> " + namedAutoMode.name);

			m_drive.zeroHeading();

			//m_drive.setAngleOffsetDegrees(namedAutoMode.getAngleOffset());
		
			Pose2d initialPose = namedAutoMode.getInitialPose();
			if (initialPose != null) {
				m_drive.resetOdometry(initialPose);
				System.out.println("Initial Pose: " + initialPose.toString());
			}
			m_autonomousCommand.schedule();
		}		
		System.out.println("autonomousInit: End");
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		CommandScheduler.getInstance().cancelAll();
		m_drive.setCurrentLimits(true);
		m_drive.resumeCSVWriter();

		initSubsystems();	

		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

	}

	public static long millis = System.currentTimeMillis();

	public static boolean doSD() {
		long now = System.currentTimeMillis();
		if (now - millis > 300) {
			millis = now;
			return true;
		}
		return false;
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		// if(doSD()){
		// SmartDashboard.putBoolean("LowSensor", m_sequencer.lowSensorHasBall());
		// SmartDashboard.putBoolean("MidSensor", m_sequencer.midSensorHasBall());
		// SmartDashboard.putBoolean("HighSensor", m_sequencer.highSensorHasBall());
		// SmartDashboard.putNumber("PowerCellCount",
		// (int)m_sequencer.getPowerCellCount());
		// SmartDashboard.putString("Intake State", m_intake.getIntakeState());
		// SmartDashboard.putNumber("Climb Encoder",
		// m_shootclimb.getClimbEncoderValue());

		// switch((int)m_sequencer.getPowerCellCount()){
		// case 1: m_ledstring.option(LedOption.BALLONE); break;
		// case 2: m_ledstring.option(LedOption.BALLTWO); break;
		// case 3: m_ledstring.option(LedOption.BALLTHREE); break;
		// case 4: m_ledstring.option(LedOption.BALLFOUR); break;
		// case 5: m_ledstring.option(LedOption.GREEN); break;
		// }
		// }
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
		SmartDashboard.putNumber("Shoot Motor % (0-1)", 0.5);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}

	@Override
	public void disabledExit(){
		m_vision.enableLED();
	}
}
