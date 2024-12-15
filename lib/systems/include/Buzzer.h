#ifndef BUZZER
#define BUZZER

/**
 * This class controls the car's buzzer. It is a singleton class (since we only have one buzzer).
 * 
 * NOTE: This class does NOT directly control the buzzer. Rather, it does the calculations of whether
 *       or not the buzzer SHOULD be active. The output of this system then goes to the Dashboard,
 *       who will activate the buzzer when this system outputs HIGH.
 */
class BuzzerController
{
public:
    static BuzzerController& getInstance()
    {
        static BuzzerController instance;
        return instance;
    }
    
    /**
     * Calling this command will activate the buzzer for the specified milliseconds. If this command
     * is called while the buzzer is still active, the timer will reset to the given value.
     */
    void activate_for_ms(unsigned long time)
    {
        _buzzer_period = time;
    }

    /**
     * Returns whether or not the buzzer should currently be active, according to the system.
     */
    bool buzzer_is_active(unsigned long millis)
    {
        return (millis - _last_activation_time_ms) < _buzzer_period;
    }

private:
    BuzzerController()
    {
        _buzzer_period = 2000; // Default is 2000ms
        _last_activation_time_ms = 0;
    }

    unsigned long _buzzer_period;
    unsigned long _last_activation_time_ms;
};

#endif /* BUZZER */
