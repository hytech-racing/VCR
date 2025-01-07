#ifndef SAFETY_SYSTEM
#define SAFETY_SYSTEM

class SafetySystem //NOLINT
{
public:
    static SafetySystem& getInstance()
    {
        static SafetySystem instance;
        return instance;
    }
 
    /**
     * Initializes AMS interface, Watchdog interface, and sets _software_is_ok to true.
     */
    void init();

    /**
     * Updates _software_is_ok by updating AMS interface and Watchdog interface. If either
     * of those have problems, then _software_is_ok goes to false.
     */
    void update_software_shutdown(unsigned long curr_millis);

    bool get_software_is_ok() {return _software_is_ok;}


private:
    SafetySystem()
    {
        _software_is_ok = false;
    }

    /* Software ok status */
    bool _software_is_ok;
};

#endif /* SAFETY_SYSTEM */
