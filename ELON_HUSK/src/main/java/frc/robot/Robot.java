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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VelocityTalonFX;
import frc.robot.subsystems.VelocityNeo;
import frc.robot.subsystems.SmartMotionNeo;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	private RobotContainer m_robotContainer;

	// The robot's subsystems
	public IntakeSubsystem m_intake;
	public VelocityTalonFX m_talon;
	public VelocityNeo m_neo;
	public SmartMotionNeo m_smart;

	public Robot() {
		addPeriodic(() -> {
		}, 0.010, 0.0); // was 0.005
	}

	private void initSubsystems() {
		// initial SubSystems to at rest states
	}


	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {

		m_intake = new IntakeSubsystem();
		m_talon = new VelocityTalonFX();
		m_neo = new VelocityNeo();
		m_smart = new SmartMotionNeo();
		
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer(m_intake, m_talon, m_neo, m_smart);

		initSubsystems();

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
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
		/*
		 * if(m_ledstring != null){ m_ledstring.option(LedOption.TEAM); }
		 */
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
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
		
		initSubsystems();
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
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}
}
