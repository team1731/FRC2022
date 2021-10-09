/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ShootClimbSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.SequencerSubsystem;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * An example command that uses an example subsystem.
 */
public class ShootAllBalls extends WaitCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShootClimbSubsystem shootSubsystem;
  private final SequencerSubsystem seqSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param intakeSubsystem The intake subsystem this command will run on
   * @param seqSubsystem The sequencer subsystem this command will run on
   */
  public ShootAllBalls(ShootClimbSubsystem shootClimbSubsystem, SequencerSubsystem sequenceSubsystem) {
    super(3);  // how many seconds to enable sequencer and keep hood open for shooting - extends WaitCommand
    shootSubsystem = shootClimbSubsystem;
    seqSubsystem = sequenceSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootClimbSubsystem, seqSubsystem);
  }

  // Called when the command is initially scheduled.
  // If it is used as Default command then it gets call all the time
  @Override
  public void initialize() {
    super.initialize();
    shootSubsystem.hoodExtend();
    shootSubsystem.enableShooting(); //turn on shoot motors
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shootMotorVelocity = shootSubsystem.getShootMotor1Velocity();
    if(shootMotorVelocity > Constants.OpConstants.kShootMinVelocity){
      seqSubsystem.shoot();
    }
    //System.out.println("ShootMotorVelocity=" + shootMotorVelocity);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("ShootAllBalls - end. interrupted=" + interrupted);
    seqSubsystem.stop();
    seqSubsystem.resetEncoder();
    shootSubsystem.hoodRetract();
  }

  /*/ Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  */
}
