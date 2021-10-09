/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SequencerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.OpConstants;

/**
 * Move the sequencer backwards until the last ball is down at the bottom (ready to intake more).
 */
public class SeqResetCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SequencerSubsystem m_SeqSubsystem;
  private Timer mTimer;
  double elapsed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param intakeSubsystem The intake subsystem this command will run on
   * @param seqSubsystem The sequencer subsystem this command will run on
   */
  public SeqResetCommand(SequencerSubsystem seqSubsystem) {
    m_SeqSubsystem = seqSubsystem;
    mTimer = new Timer();
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(seqSubsystem);
  }

  // Called when the command is initially scheduled.
  // If it is used as Default command then it gets call all the time
  @Override
  public void initialize() {
    //m_SeqSubsystem.stop();
    mTimer.start();
    elapsed = mTimer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if mid sensor has not tripped then reset balls back to mid sensor (tripped)
    if(m_SeqSubsystem.midSensorHasBall()) {
      m_SeqSubsystem.stop();
    } else {
      m_SeqSubsystem.reverse();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("SeqResetCommand end interrupted=" + (interrupted?"true":"false"));
    m_SeqSubsystem.stop();
    mTimer.stop();
  }

  // Returns true when the command should end after 2 seconds.
  @Override
  public boolean isFinished() {
    boolean result = false;
    if (mTimer.get() - elapsed > OpConstants.kSeqResetDelay) {
      result = true;
    }
    return result;
  }
}
