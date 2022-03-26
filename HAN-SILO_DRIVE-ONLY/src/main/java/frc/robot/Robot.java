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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OpConstants.LedOption;
import edu.wpi.first.math.geometry.Pose2d;

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
	public DriveSubsystem m_robotDrive;

	public Robot() {
		addPeriodic(() -> {
			if (m_robotDrive != null)
				m_robotDrive.updateOdometry();
		}, 0.020, 0.0); // was 0.005
	}

	private void initSubsystems() {
		// initial SubSystems to at rest states
		m_robotDrive.resetEncoders();
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {

		m_robotDrive = new DriveSubsystem();

		m_robotDrive.zeroHeading();

		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer(m_robotDrive);

		initSubsystems();

		try {
			File branchInfo = new File(Filesystem.getDeployDirectory() + "/DeployedBranchInfo~.txt");
			Scanner reader = new Scanner(branchInfo);
			String fullText = "";
			while (reader.hasNext()) {
				fullText += reader.nextLine();
			}
			SmartDashboard.putString("Build Info", fullText);
			reader.close();
		} catch (FileNotFoundException fnf) {
			SmartDashboard.putString("Build Info", "N/A");
			System.err.println("DeployedBranchInfo~.txt not found");
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

		m_robotDrive.displayEncoders();
	}

	@Override
	public void disabledPeriodic() {
		m_robotDrive.resetEncoders();
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		CommandScheduler.getInstance().cancelAll();
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

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}
}
