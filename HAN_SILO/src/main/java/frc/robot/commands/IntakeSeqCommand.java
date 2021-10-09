/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* Intake Motor (I), Sequence Motor (S)
  Lo Mid Hi | iU/D iR Seq
   0  0  0  |   D   R  Off  ==> case A
   0  0  1  |   D   R  Off  ==> case A
   0  1  0  |   D   -  On   ==> case B
   0  1  1  |   D   R  Off  ==> case A
   1  0  0  |   D   -  On   ==> case B
   1  0  1  |   U   -  Off  ==> case C
   1  1  0  |   D   -  On   ==> case B
   1  1  1  |   U   -  Off  ==> case C

   If you switch case 2 & 3, then spacing will be ball width, now it's sensor width
*/
package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class IntakeSeqCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_IntakeSubsystem;
  private final SequencerSubsystem m_SeqSubsystem;
  private boolean low;
  private boolean mid;
  private boolean high;
 // private Intake1ball sequenceBallCommand;
  //private boolean interruptible;
  //private boolean intakingBallNow = false;

  /**
   * use this constructor for autonomous and set autonomous=true!
   */
  public IntakeSeqCommand(IntakeSubsystem intakeSubsystem, SequencerSubsystem seqSubsystem, boolean autonomous) {
    m_IntakeSubsystem = intakeSubsystem;
    m_SeqSubsystem = seqSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, seqSubsystem);
    //this.interruptible = autonomous;

  }

  /**
   * use this constructor for teleop
   */
  public IntakeSeqCommand(IntakeSubsystem intakeSubsystem, SequencerSubsystem seqSubsystem) {
    this(intakeSubsystem, seqSubsystem, false);
  }

  // Called when the command is initially scheduled.
  // If it is used as Default command then it gets call all the time
  @Override
  public void initialize() {
    m_IntakeSubsystem.extend();
    //m_SeqSubsystem.stop();
    m_SeqSubsystem.enableInterrupts();
    if (!m_SeqSubsystem.isIntakingBall()) {
      m_SeqSubsystem.resetEncoder();
    }
//    sequenceBallCommand = new Intake1ball(m_SeqSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    low = m_SeqSubsystem.lowSensorHasBall();
    mid = low;  //use low for both mid and low
    //mid = m_SeqSubsystem.midSensorHasBall();
    high = m_SeqSubsystem.highSensorHasBall();
    if (!low && ((!mid && !high) || (!mid && high) || (mid && high))) {  // case A
      m_IntakeSubsystem.extend();
      m_IntakeSubsystem.active();
    //  m_SeqSubsystem.stop();
    } else if (!high && ((!low && mid) || (low && !mid) || (low && mid))) { // case B
      m_IntakeSubsystem.extend();
      m_IntakeSubsystem.inactive();
   //   m_SeqSubsystem.forward(false);;
    } else if (low && high) { // case C
      m_IntakeSubsystem.retract();
      m_IntakeSubsystem.inactive();
   //   m_SeqSubsystem.stop();
    }

    if (low ) {
    //  sequenceBallCommand.withTimeout(2);
      m_SeqSubsystem.setIntakingBall(true);
    }
    if (m_SeqSubsystem.atSetpoint() || m_SeqSubsystem.highSensorHasBall()) {
        m_SeqSubsystem.setIntakingBall(false);
        m_SeqSubsystem.resetEncoder();
        m_SeqSubsystem.stop();
    } else if (m_SeqSubsystem.isIntakingBall()) {
      m_SeqSubsystem.intakeBall(); // interruptible must be true for autonomous
    }
  }


    
    
    /* if low and high not tripped do something
    // if they are both tripped we do NOTHING
    if(!m_SeqSubsystem.lowSensorHasBall() && !m_SeqSubsystem.highSensorHasBall() ){
      //do something
      if (m_SeqSubsystem.lowSensorHasBall() ||  m_SeqSubsystem.midSensorHasBall()) {
        m_IntakeSubsystem.inactive();
      } else {
        m_IntakeSubsystem.active();
      }
      System.out.println("intake extended");
    }
    else if (m_SeqSubsystem.lowSensorHasBall() && m_SeqSubsystem.highSensorHasBall()){
      m_IntakeSubsystem.inactive();
      m_IntakeSubsystem.retract();
    }
    if((m_SeqSubsystem.lowSensorHasBall() || m_SeqSubsystem.midSensorHasBall()) && !m_SeqSubsystem.highSensorHasBall()){
      m_SeqSubsystem.forward(false); // false indicates forward intaking speed (i.e., NOT shooting)
    }
    else{
      m_SeqSubsystem.stop();
    }
    */
  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("IntakeSequenceCommand end interrupted=" + (interrupted?"true":"false"));
    m_SeqSubsystem.stop();
    m_IntakeSubsystem.inactive();
    m_IntakeSubsystem.retract();
    m_SeqSubsystem.disableInterrupts();
 //   sequenceBallCommand = null;
    System.out.println("in end of intake sequence, interrupted=" + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //m_SeqSubsystem.highSensorHasBall(); //m_SeqSubsystem.getMaxPowerCells();
  }
}
