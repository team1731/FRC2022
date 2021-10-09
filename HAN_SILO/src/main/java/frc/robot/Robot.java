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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OpConstants.LedOption;
import frc.robot.autonomous._NamedAutoMode;
import frc.robot.subsystems.ColorWheelSubsystem;
import frc.robot.subsystems.LedStringSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import frc.robot.subsystems.ShootClimbSubsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;

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
  private Integer fieldOrientation;

  // The robot's subsystems
  public DriveSubsystem m_robotDrive;
  public LimeLightSubsystem m_vision;
  public IntakeSubsystem m_intake;
  public SequencerSubsystem m_sequencer;
  public ShootClimbSubsystem m_shootclimb;
  public ColorWheelSubsystem m_colorwheel;
  public LedStringSubsystem m_ledstring;

  String autoCode = AutoConstants.kDEFAULT_AUTO_CODE;

  public Robot() {
     addPeriodic(() -> {
       if(m_robotDrive != null) m_robotDrive.updateOdometry();
     }, 0.005, 0.0);
  }


  private void initSubsystems() {
    // initial SubSystems to at rest states
    m_robotDrive.resetEncoders();
    m_intake.retract();
    m_sequencer.stop();
    m_shootclimb.stopShooting();
    // m_colorwheel.init();
    // LEDs are not attached currently
    // m_ledstring.init();
  }

  private void autoInitPreload() {
    System.out.println("autoInitPreload: Start");
    m_autonomousCommand = null;
    m_robotDrive.resetOdometry(new Pose2d());
    m_ledstring.option(LedOption.RAINBOW);

    m_robotDrive.resumeCSVWriter();
    m_sequencer.setPowerCellCount((int) SmartDashboard.getNumber("INIT CELL COUNT", 3));

    if (RobotBase.isReal()) {
      autoCode = SmartDashboard.getString("AUTO CODE", autoCode);
    }
    System.out.println("AUTO CODE retrieved from Dashboard --> " + autoCode);
    if (autoCode == null || autoCode.length() < 2) {
      autoCode = AutoConstants.kDEFAULT_AUTO_CODE;
    }
    autoCode = autoCode.toUpperCase();
    System.out.println("AUTO CODE being used by the software --> " + autoCode);

    m_autonomousCommand = null;
    namedAutoMode = m_robotContainer.getNamedAutonomousCommand(autoCode);
    if (namedAutoMode != null) {
      System.out.println("autoInitPreload: getCommand Auto Begin");
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

    CameraServer camServer = CameraServer.getInstance();
    camServer.startAutomaticCapture();

    m_ledstring = new LedStringSubsystem();
    m_vision = new LimeLightSubsystem();
    m_robotDrive = new DriveSubsystem(m_vision);
    m_intake = new IntakeSubsystem(/*m_ledstring*/);
    m_sequencer = new SequencerSubsystem(m_ledstring);
    m_shootclimb = new ShootClimbSubsystem(m_ledstring);
    m_colorwheel = null; // new ColorWheelSubsystem();

    m_robotDrive.zeroHeading();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(m_ledstring, m_robotDrive, m_intake, m_sequencer, m_shootclimb, m_vision);

    initSubsystems();

    SmartDashboard.putString("INIT CELL COUNT", "3"); // How much ammo we start with

    SmartDashboard.putString("AUTO CODE", AutoConstants.kDEFAULT_AUTO_CODE); // see above for explanation
    if (RobotBase.isReal()) {
      autoCode = SmartDashboard.getString("AUTO CODE", autoCode);
    }

    autoInitPreload();

    SmartDashboard.putBoolean("Vis_HasTarget", false);
    SmartDashboard.putNumber("Vis_TargetAngle", 0);
    
    //Report Build Information to the SmartDashboard
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

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_robotDrive.suspendCSVWriter();
  }

  @Override
  public void disabledPeriodic() {
    m_ledstring.option(LedOption.TEAM);
    m_robotDrive.resetEncoders();
    if (System.currentTimeMillis() % 100 == 0) {
      SmartDashboard.putBoolean("LowSensor", m_sequencer.lowSensorHasBall());
      SmartDashboard.putBoolean("MidSensor", m_sequencer.midSensorHasBall());
      SmartDashboard.putBoolean("HighSensor", m_sequencer.highSensorHasBall());
    }

    if (RobotBase.isReal()) {
      String newCode = SmartDashboard.getString("AUTO CODE", autoCode);
      if (!newCode.equals(autoCode)) {
        autoCode = newCode;
        System.out.println("New Auto Code read from dashboard - initializing.");
        autoInitPreload();
      }
      if(m_autonomousCommand != null){
        if(m_autonomousCommand.getName().startsWith("H0")){
          Integer newFieldOrientation = namedAutoMode.getFieldOrientation();
          if(newFieldOrientation != null){
            if(!newFieldOrientation.equals(fieldOrientation)){
              System.out.println("New Field Orientation detected by LimeLight - initializing.");
              fieldOrientation = newFieldOrientation;
              autoInitPreload();
            }
          }
        }
      }
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_ledstring.option(LedOption.RAINBOW);
    m_robotDrive.resumeCSVWriter();
    m_sequencer.setPowerCellCount((int) SmartDashboard.getNumber("INIT CELL COUNT", 3));

    CommandScheduler.getInstance().cancelAll();

    // schedule the autonomous command (example)
    if (m_autonomousCommand == null) {
      System.err.println("SOMETHING WENT WRONG - UNABLE TO RUN AUTONOMOUS! CHECK SOFTWARE!");
    } else {
      System.out.println("Running actual autonomous mode --> " + namedAutoMode.name);
      m_robotDrive.zeroHeading();
      Pose2d initialPose = namedAutoMode.getInitialPose();
      if(initialPose != null){
        m_robotDrive.resetOdometry(initialPose);
        System.out.println("Initial Pose: "+initialPose.toString());
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
    m_sequencer.setPowerCellCount((int) SmartDashboard.getNumber("INIT CELL COUNT", 3));
    m_robotDrive.resumeCSVWriter();

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
  public static boolean doSD(){
    long now = System.currentTimeMillis();
    if(now - millis > 300){
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
    if(doSD()){
      SmartDashboard.putBoolean("LowSensor",  m_sequencer.lowSensorHasBall());
      SmartDashboard.putBoolean("MidSensor",  m_sequencer.midSensorHasBall());
      SmartDashboard.putBoolean("HighSensor",  m_sequencer.highSensorHasBall());
      SmartDashboard.putNumber("PowerCellCount",  (int)m_sequencer.getPowerCellCount());
      SmartDashboard.putString("Intake State",  m_intake.getIntakeState());
      //SmartDashboard.putNumber("Climb Encoder", m_shootclimb.getClimbEncoderValue());

    // switch((int)m_sequencer.getPowerCellCount()){
    //   case 1: m_ledstring.option(LedOption.BALLONE); break;
    //   case 2: m_ledstring.option(LedOption.BALLTWO); break;
    //   case 3: m_ledstring.option(LedOption.BALLTHREE); break;
    //   case 4: m_ledstring.option(LedOption.BALLFOUR); break;
    //   case 5: m_ledstring.option(LedOption.GREEN); break;
    // }
    }
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
    double shootMotorPercent_0_to_1 = SmartDashboard.getNumber("Shoot Motor % (0-1)", 0.5);
    m_shootclimb.hoodExtend();
    m_shootclimb.spinShooter(shootMotorPercent_0_to_1);
    //SmartDashboard.putNumber("Shoot Motor 1 Vel", m_shootclimb.getShootMotor1Velocity());
  }
}
