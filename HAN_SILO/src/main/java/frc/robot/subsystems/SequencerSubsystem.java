/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OpConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

//import edu.wpi.first.wpilibj.Timer;

public class SequencerSubsystem extends SubsystemBase {

  //private final LedStringSubsystem m_ledstring;
  private final TalonFX mTalonSeq;
  private DigitalInput mLowSensor;
  private DigitalInput mMidSensor;
  private DigitalInput mHighSensor;
  //private Timer mTimer;
  //private double elapsed;
  //private boolean startDelay;
  //private boolean mLowSensorCur;
  private boolean mLastLowHasBall; // does robot want to index balls - mode
  private boolean mLastHighHasBall; // does robot want to index balls - mode
  private int mPowerCellCount;
  private int targetTicks = 20000;
  private boolean mIsIntakingBall = false;

  /**
   * Creates a new SequencerSubsystem.
 * @param m_ledstring
   */
  public SequencerSubsystem(LedStringSubsystem m_ledstring) {
    //this.m_ledstring = m_ledstring;
    mTalonSeq = new TalonFX(OpConstants.kMotorSeq);
    mTalonSeq.configFactoryDefault();
    mTalonSeq.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, OpConstants.kPIDLoopIdx, OpConstants.kTimeoutMs);
		mTalonSeq.configNominalOutputForward(0, OpConstants.kTimeoutMs);
		mTalonSeq.configNominalOutputReverse(0, OpConstants.kTimeoutMs);
		mTalonSeq.configPeakOutputForward(1, OpConstants.kTimeoutMs);
    mTalonSeq.configPeakOutputReverse(-1, OpConstants.kTimeoutMs);
    mTalonSeq.config_kF(OpConstants.kPIDLoopIdx, 0.07, OpConstants.kTimeoutMs);
		mTalonSeq.config_kP(OpConstants.kPIDLoopIdx, 0.05, OpConstants.kTimeoutMs);
		mTalonSeq.config_kI(OpConstants.kPIDLoopIdx, 0, OpConstants.kTimeoutMs);
    mTalonSeq.config_kD(OpConstants.kPIDLoopIdx, 0, OpConstants.kTimeoutMs);
    mTalonSeq.configMotionCruiseVelocity(12000);
    mTalonSeq.configMotionAcceleration(20000);
    mTalonSeq.setInverted(true);
    
    
    mLowSensor = new DigitalInput(OpConstants.kLowSequencer);
    mMidSensor = new DigitalInput(OpConstants.kMidSequencer);
    mHighSensor = new DigitalInput(OpConstants.kHighSequencer);
/*
    mMidSensor.requestInterrupts(new InterruptHandlerFunction<Object>() {
		@Override
		public void interruptFired(int interruptAssertedMask, Object param) {
      //mTalonSeq.set(0);
      stop();
      System.out.println("Mid Interrupt Received: stopping sequencer");
    }});
    mMidSensor.setUpSourceEdge(false, true);

    mHighSensor.requestInterrupts(new InterruptHandlerFunction<Object>() {
      @Override
      public void interruptFired(int interruptAssertedMask, Object param) {
        //mTalonSeq.set(0);
        stop();
        System.out.println("High Interrupt Received: stopping sequencer");
      }});
    mHighSensor.setUpSourceEdge(true, true);
    */
    //mTimer = new Timer();
    //mTimer.start();
    //startDelay = false;
    //mLowSensorCur = mLowSensor.get();
    mLastLowHasBall = lowSensorHasBall();
    mLastHighHasBall = highSensorHasBall();
    mPowerCellCount = 0;
    mTalonSeq.setSelectedSensorPosition(0);
   
  }

  public void enableInterrupts(){
    //mMidSensor.enableInterrupts();
    //mHighSensor.enableInterrupts();
  }

  public void disableInterrupts(){
    //mMidSensor.disableInterrupts();
    //mHighSensor.disableInterrupts();
  }

  public void setPowerCellCount(int numBalls){
    mPowerCellCount = numBalls;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("seqencoder", mTalonSeq.getSelectedSensorPosition());
    // Incrementing ball count
    if (lowSensorHasBall()) {
      if (!mLastLowHasBall) {
        if (mPowerCellCount <= OpConstants.kMaxPowerCells) {
          mPowerCellCount++;
        }
      }
    }
    // Decrementing ball count
    if (!highSensorHasBall()) {
      if (mLastHighHasBall) {
        if (mPowerCellCount > 0) {
          mPowerCellCount--;
        }
      }
    }

    mLastLowHasBall = lowSensorHasBall();
    mLastHighHasBall = highSensorHasBall();

   // System.out.println(mTalonSeq.getSelectedSensorPosition());
  }
  
  /**
   * For Shooting: Enables the Sequencer by turning on motor.
   */
  /*
  public void forward(boolean shooting) {
    if(shooting){
      mTalonSeq.setSpeed(OpConstants.kMotorSeqFwdShootSpeed);
    }
    else{
      mTalonSeq.setSpeed(OpConstants.kMotorSeqFwdIntakeSpeed);
    }
    //mPowerCellCount = 0;
  }
*/
  public void shoot() {

     // mTalonSeq.set(ControlMode.Velocity,1000.0 * 2048 / 600);
     mTalonSeq.set(ControlMode.PercentOutput, 0.4);
    //mPowerCellCount = 0;
  }
  /**
   * For Ejecting balls: Reverses the motor.
   */
  public void reverse() {
    mTalonSeq.set(ControlMode.PercentOutput,-0.4);
    mPowerCellCount = 0;
  }

  /**
   * Enables the intake by retracting solenoid & turning off motor.
   */
  public void stop() {
    mTalonSeq.set(ControlMode.PercentOutput, 0);
 
  }

  public boolean lowSensorHasBall() {
    return !mLowSensor.get();
  }

  public boolean midSensorHasBall() {
    return !mMidSensor.get();
  }

  public boolean highSensorHasBall() {
    return !mHighSensor.get();
  }

  public boolean getMaxPowerCells() {
    return(mPowerCellCount >= OpConstants.kMaxPowerCells);
  }

  public int getPowerCellCount() {
    return(mPowerCellCount);
  }

public void resetEncoder() {
  mTalonSeq.setSelectedSensorPosition(0);
}

public void intakeBall() {
 // mTalonSeq.set(ControlMode.Position, targetTicks);
  mTalonSeq.set(ControlMode.MotionMagic, targetTicks);
}

public boolean atSetpoint() {
	return Math.abs(targetTicks - mTalonSeq.getSelectedSensorPosition())  < 1000;
}

public double getencoder() {
	return mTalonSeq.getSelectedSensorPosition();
}

public void setIntakingBall(boolean b) {
  mIsIntakingBall = b;
}


public boolean isIntakingBall() {
	return mIsIntakingBall;
}

  /*
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mLowSensorCur = mLowSensor.get();
    if (startDelay) {
      if (mTimer.get() - elapsed > OpConstants.kSeqIntakeDelay) {
        mTalonSeq.setSpeed(0);
        startDelay = false;
      }
    }
  }

  public void addPowerCell() {
    if (mLowSensorCur) {
        startDelay = true;
        elapsed = mTimer.get();
        // incr count at end of intaking powercell
        if (!mLowSensorLast) {
          mPowerCellCount++;
        }
    } else {
        if (mPowerCellCount < OpConstants.kMaxPowerCells) {
            mTalonSeq.setSpeed(OpConstants.kMotorSeqFwdIntakeSpeed);
            // incr count at beginning of intaking powercell
            //if (mLowSensorLast) {
            //    mPowerCellCount++;
            //}
        }
    }
    mLowSensorLast = mLowSensorCur;
  }
  */

}
