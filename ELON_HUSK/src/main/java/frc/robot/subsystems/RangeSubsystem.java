// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


public class RangeSubsystem extends ToggleableSubsystem {

  //#region ToggleableSubsystem
	@Override
	protected boolean getEnabled(){
		return false;
	}
	//#endregion

  /** Creates a new ExampleSubsystem. */
  public RangeSubsystem() {
		if(isDisabled()){ return; }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isDisabled()){ return; }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
