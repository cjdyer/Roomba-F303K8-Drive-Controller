#include <motorClass.h>
#include <Arduino.h>

tachoWheel::tachoWheel(){
    
}

void tachoWheel::encoderTick(){
    PeriodBetweenPulses   = micros() - LastTimeWeMeasured;
    LastTimeWeMeasured    = micros();

    if(PulseCounter >= AmountOfReadings){  // If counter for amount of readings reach the set limit:
    
        PeriodAverage  = PeriodSum / AmountOfReadings;
        PulseCounter   = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
        PeriodSum      = PeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.

        int RemapedAmountOfReadingsL = map(PeriodBetweenPulses, 40000, 5000, 1, 10);  // Remap the period range to the reading range.
                                    // 1st value is what are we going to remap. In this case is the PeriodBetweenPulses.
                                    // 2nd value is the period value when we are going to have only 1 reading. The higher it is, the lower RPM has to be to reach 1 reading.
                                    // 3rd value is the period value when we are going to have 10 readings. The higher it is, the lower RPM has to be to reach 10 readings.
                                    // 4th and 5th values are the amount of readings range.
        RemapedAmountOfReadingsL = constrain(RemapedAmountOfReadingsL, 1, 10);  // Constrain the value so it doesn't go below or above the limits.
        AmountOfReadings = RemapedAmountOfReadingsL;  // Set amount of readings as the remaped value.
    
    }else{
        PulseCounter++;  // Increase the counter for amount of readings by 1.
        PeriodSum = PeriodSum + PeriodBetweenPulses;  // Add the periods so later we can average.
    }
}

void tachoWheel::calcVelocity(){
    // The following is going to store the two values that might change in the middle of the cycle.
    // We are going to do math and functions with those values and they can create glitches if they change in the
    // middle of the cycle.
    LastTimeCycleMeasure = LastTimeWeMeasured;  // Store the LastTimeWeMeasured in a variable.
    CurrentMicros = micros();  // Store the micros() in a variable.

    // CurrentMicros should always be higher than LastTimeWeMeasured, but in rare occasions that's not true.
    // I'm not sure why this happens, but my solution is to compare both and if CurrentMicros is lower than
    // LastTimeCycleMeasure I set it as the CurrentMicros.
    // The need of fixing this is that we later use this information to see if pulses stopped.
    if(CurrentMicros < LastTimeCycleMeasure){
    LastTimeCycleMeasure = CurrentMicros;
    }

    // Calculate the frequency:
    FrequencyRaw = 10000000000 / PeriodAverage;  // Calculate the frequency using the period between pulses.

    // Detect if pulses stopped or frequency is too low, so we can show 0 Frequency:
    if(PeriodBetweenPulses > ZeroTimeout          - ZeroDebouncingExtra || 
    CurrentMicros       - LastTimeCycleMeasure > ZeroTimeout         - ZeroDebouncingExtra){
        // If the pulses are too far apart that we reached the timeout for zero:
        FrequencyRaw = 0;  // Set frequency as 0.
        ZeroDebouncingExtra = 2000;  // Change the threshold a little so it doesn't bounce.

    } else {
        ZeroDebouncingExtra = 0;  // Reset the threshold to the normal value so it doesn't bounce.
    }

    FrequencyReal = FrequencyRaw / 10000;  // Get frequency without decimals.
                                    // This is not used to calculate RPM but we remove the decimals just in case
                                    // you want to print it.

    // Calculate the RPM:
    RPM = FrequencyRaw / PulsesPerRevolution * 60;  // Frequency divided by amount of pulses per revolution multiply by
                                    // 60 seconds to get minutes.
    RPM = RPM / 10000;  // Remove the decimals.

    // Smoothing RPM:
    total = total - readings[readIndex];  // Advance to the next position in the array.
    readings[readIndex] = RPM;            // Takes the value that we are going to smooth.
    total = total + readings[readIndex];  // Add the reading to the total.
    readIndex = readIndex + 1;            // Advance to the next position in the array.

    if(readIndex >= numReadings){ // If we're at the end of the array:
    readIndex = 0;  // Reset array index.
    }

    // Calculate the average:
    average = total / numReadings;  // The average value it's the smoothed result.
}