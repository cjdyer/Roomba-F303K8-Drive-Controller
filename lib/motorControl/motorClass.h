#include <Arduino.h>

class tachoWheel{
    public:
        tachoWheel();
        void            calcVelocity();
        void            encoderTick();
        
        unsigned long   getVelocity(){return average;}

        const uint16_t          PulsesPerRevolution = 1633; // Encoder pulses for 1 full rotation (Quadrature)
        const unsigned long     ZeroTimeout = 100000;       // If the period between pulses is too high, or even if the pulses stopped, then we would get stuck showing the
                                                            // last value instead of a 0. Because of this we are going to set a limit for the maximum period allowed.
                                                            // If the period is above this value, the RPM will show as 0.
                                                            // The higher the set value, the longer lag/delay will have to sense that pulses stopped, but it will allow readings
                                                            // at very low RPM.
                                                            // Setting a low value is going to allow the detection of stop situations faster, but it will prevent having low RPM readings.
                                                            // The unit is in microseconds.
        const byte              numReadings = 10;            // Number of samples for smoothing. The higher, the more smoothing, but it's going to
                                                            // react slower to changes. 1 = no smoothing. Default: 2.
        volatile unsigned long  LastTimeWeMeasured;         // Time at the last pulse
        volatile unsigned long  PeriodBetweenPulses = ZeroTimeout+1000; // Stores the period between pulses in microseconds.
                                                                        // It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
        volatile unsigned long  PeriodAverage = ZeroTimeout+1000;       // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.

        unsigned long   CurrentMicros = micros(); // Micros at the current cycle
        unsigned long   FrequencyRaw;             // Calculated frequency, based on the period. This has a lot of extra decimals without the decimal point.
        unsigned long   FrequencyReal;            // Frequency without decimals.
        unsigned long   PeriodSum;                // Stores the summation of all the periods to do the average.
        unsigned long   RPM;                      // Raw RPM without any processing.
        unsigned long   LastTimeCycleMeasure = LastTimeWeMeasured; // Stores the last time we measure a pulse in that cycle.
                                            // We need a variable with a value that is not going to be affected by the interrupt
                                            // because we are going to do math and functions that are going to mess up if the values
                                            // changes in the middle of the cycle.
        unsigned int    PulseCounter = 1;         // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.
        unsigned int    AmountOfReadings = 1;     // Stores the last time we measure a pulse in that cycle.
                                                    // We need a variable with a value that is not going to be affected by the interrupt
                                                    // because we are going to do math and functions that are going to mess up if the values
                                                    // changes in the middle of the cycle.
        unsigned int    ZeroDebouncingExtra;      // Stores the extra value added to the ZeroTimeout to debounce it.

        // Variables for smoothing tachometer:
        unsigned long readings[10];  // The input. // [numReadings]
        unsigned long readIndex;  // The index of the current reading.
        unsigned long total;  // The running total.
        unsigned long average;  // The RPM value after applying the smoothing.
};