package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
// import edu.wpi.first.wpilibj.AnalogTriggerOutput.AnalogTriggerType;
// import edu.wpi.first.wpilibj.Counter;
import frc.robot.Constants.OpConstants;

/**
 * Driver for an analog Sharp IR sensor (or any distance sensor where output
 * voltage is a function of range, really).
 */
public class IRSensor {
    protected final AnalogInput mAnalogInput;
    protected final AnalogTrigger mAnalogTrigger;
    // protected final Counter mCounter;

    public IRSensor(int port) {
        //mAnalogInput = new AnalogInput(port);
        mAnalogInput = new AnalogInput(getChannelFromPin( PinType.AnalogIn, port));
        mAnalogInput.setAverageBits(6);
        mAnalogTrigger = new AnalogTrigger(mAnalogInput);
        mAnalogTrigger.setAveraged(true);
        // mAnalogTrigger.setFiltered(false);
        // mAnalogTrigger.setLimitsVoltage(OpConstants.kMinIRVoltage,
        // OpConstants.kMaxIRVoltage);
        mAnalogTrigger.setLimitsVoltage(1.0, 1.5);
        // mCounter = new
        // Counter(mAnalogTrigger.createOutput(AnalogTriggerType.kState));
    }

    // public int getCount() {
    // return mCounter.get();
    // }

    public double getVoltage() {
        return mAnalogInput.getAverageVoltage();
    }

    public boolean isTriggered() {
        return mAnalogTrigger.getTriggerState();
    }

    public enum PinType { DigitalIO, PWM, AnalogIn, AnalogOut };
    
    public final int MAX_NAVX_MXP_DIGIO_PIN_NUMBER      = 9;
    public final int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER   = 3; //should only use 0 & 1
    public final int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER  = 1;
    public final int NUM_ROBORIO_ONBOARD_DIGIO_PINS     = 10;
    public final int NUM_ROBORIO_ONBOARD_PWM_PINS       = 10;
    public final int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS  = 4;
    
    /* getChannelFromPin( PinType, int ) - converts from a navX-MXP */
    /* Pin type and number to the corresponding RoboRIO Channel     */
    /* Number, which is used by the WPI Library functions.          */
    
    public int getChannelFromPin( PinType type, int io_pin_number ) 
               throws IllegalArgumentException {
        int roborio_channel = 0;
        if ( io_pin_number < 0 ) {
            throw new IllegalArgumentException("Error:  navX-MXP I/O Pin #");
        }
        switch ( type ) {
        case DigitalIO:
            if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {
                throw new IllegalArgumentException("Error:  Invalid navX-MXP Digital I/O Pin #");
            }
            roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_DIGIO_PINS + 
                              (io_pin_number > 3 ? 4 : 0);
            break;
        case PWM:
            if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {
                throw new IllegalArgumentException("Error:  Invalid navX-MXP Digital I/O Pin #");
            }
            roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_PWM_PINS;
            break;
        case AnalogIn:
            if ( io_pin_number > MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER ) {
                throw new IllegalArgumentException("Error:  Invalid navX-MXP Analog Input Pin #");
            }
            roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_ANALOGIN_PINS;
            break;
        case AnalogOut:
            if ( io_pin_number > MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER ) {
                throw new IllegalArgumentException("Error:  Invalid navX-MXP Analog Output Pin #");
            }
            roborio_channel = io_pin_number;            
            break;
        }
        return roborio_channel;
    }
}
