/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.Constants.OpConstants;
import frc.robot.subsystems.ShootClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ClimbingCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShootClimbSubsystem m_ShootClimbSubsystem;
  
  private final DoubleSupplier climb;

  boolean isHiCy;
  boolean isLoCy;
  boolean isClimbEx;
  boolean isClimbRt;

  boolean isCyExtending;
  boolean isCyRetracting;

  /**
   * Creates a new ExampleCommand.
   *
   * @param ShootClimbSubsystem The intake subsystem this command will run on
   * @param seqSubsystem The sequencer subsystem this command will run on
   */
  public ClimbingCommand(ShootClimbSubsystem shootClimbSubsystem, DoubleSupplier climb) {
    m_ShootClimbSubsystem = shootClimbSubsystem;  
    this.climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootClimbSubsystem);
  }

  // Called when the command is initially scheduled.
  // If it is used as Default command then it gets call all the time
  @Override
  public void initialize() {
    //m_ShootClimbSubsystem.disable(); Can't call this all the time it will make it ineffective
    isCyExtending = false;
    isCyRetracting = false;
    m_ShootClimbSubsystem.resetClimbEncoder();
    m_ShootClimbSubsystem.climbRetract();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    isHiCy = m_ShootClimbSubsystem.isHiCylinderSensor();
    isLoCy = m_ShootClimbSubsystem.isLoCylinderSensor();
    isClimbEx = m_ShootClimbSubsystem.isClimbExtendSensor();
    isClimbRt = m_ShootClimbSubsystem.isClimbRetractSensor();
    
    double climbPercent = climb.getAsDouble();
    // if within Joystick deadband then set output to Zero
    if (Math.abs(climbPercent) < OpConstants.kJoystickDeadband) {
      climbPercent = 0;
      m_ShootClimbSubsystem.brakeOn();
    } else {
      m_ShootClimbSubsystem.brakeOff();
    }
    
    if (climbPercent > 0 && !isCyExtending) {
      m_ShootClimbSubsystem.climbExtend();
      isCyExtending = true;
      isCyRetracting = false;
    } else if (climbPercent < 0) {
      if(isLoCy){
        m_ShootClimbSubsystem.climbRetract();
      }
      if(!isCyRetracting){
        isCyRetracting = true;
        isCyExtending = false;
      }
    }
        //EXTENDING    NOT REACHED LIMIT    RETRACTING     NOT REACHED LIMIT
    if ((isCyExtending && !isClimbEx && (
             (m_ShootClimbSubsystem.getClimbEncoderValue() < OpConstants.kClimbExSafeEncValue) 
               || isHiCy) ) 
         ||
        (isCyRetracting && !isClimbRt)) { //WADE: how to use isLoCy ????
          m_ShootClimbSubsystem.setClimber(climbPercent * OpConstants.kClimbJoystickInvert);
    } else {
      m_ShootClimbSubsystem.setClimber(0);
    }

  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShootClimbSubsystem.stopShooting();
    m_ShootClimbSubsystem.brakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
