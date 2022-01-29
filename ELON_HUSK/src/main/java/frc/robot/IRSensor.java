package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
// import edu.wpi.first.wpilibj.AnalogTriggerOutput.AnalogTriggerType;
// import edu.wpi.first.wpilibj.Counter;
import frc.robot.Constants.OpConstants;

/**
 * Driver for an analog Sharp IR sensor (or any distance sensor where output voltage is a function of range, really).
 */
public class IRSensor {
    protected final AnalogInput mAnalogInput;
    protected final AnalogTrigger mAnalogTrigger;
    // protected final Counter mCounter;

    public IRSensor(int port) {
        mAnalogInput = new AnalogInput(port);
        mAnalogInput.setAverageBits(6);
        mAnalogTrigger = new AnalogTrigger(mAnalogInput);
        mAnalogTrigger.setAveraged(true);
        mAnalogTrigger.setFiltered(false);
        mAnalogTrigger.setLimitsVoltage(OpConstants.kMinIRVoltage, OpConstants.kMaxIRVoltage);
        // mCounter = new Counter(mAnalogTrigger.createOutput(AnalogTriggerType.kState));
    }

    // public int getCount() {
    //     return mCounter.get();
    // }

    public double getVoltage() {
        return mAnalogInput.getAverageVoltage();
    }

    public boolean isTriggered() {
        return mAnalogTrigger.getTriggerState();
    }

    // public void resetCount() {
    //     mCounter.reset();
    // }
}
