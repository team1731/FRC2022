/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.subsystems.SequencerSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ShootSeqCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShootClimbSubsystem shootSubsystem;
  private final SequencerSubsystem seqSubsystem;
  //private double startTime;

  /**
   * Creates a new ExampleCommand.
   *
   * @param intakeSubsystem The intake subsystem this command will run on
   * @param seqSubsystem The sequencer subsystem this command will run on
   */
  public ShootSeqCommand(ShootClimbSubsystem shoot, SequencerSubsystem seq) {
    shootSubsystem = shoot;
    seqSubsystem = seq;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(seqSubsystem);
  }

  // Called when the command is initially scheduled.
  // If it is used as Default command then it gets call all the time
  @Override
  public void initialize() {
    //startTime = Timer.getFPGATimestamp();
    //System.out.println("starttime = " + startTime);
    //shootSubsystem.hoodExtend();
    //seqSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double shootMotorVelocity = shootSubsystem.getShootMotor1Velocity();
    //System.out.println("shoot motor velocity = " + shootMotorVelocity);
    if(shootSubsystem.atTargetVelocity()){
      //System.out.println("calling seqSubSystem.forward(true);");
      seqSubsystem.shoot();
    }
    else{

      //System.out.println("waiting for shoot motor to come up to speed");
    }
    // get necessary input
    //if (!m_SeqSubsystem.getMaxPowerCells()) {
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    seqSubsystem.stop();
    seqSubsystem.resetEncoder();
    //shootSubsystem.stopShooting();
    //shootSubsystem.hoodRetract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    double curTime = Timer.getFPGATimestamp();
    double elapsed = curTime - startTime;
    System.out.println("curtime = " + curTime + ", elapsed=" + elapsed);
    return  elapsed >= 1;
    */
    return false;
  }
}
