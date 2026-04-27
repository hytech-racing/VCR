enum ADSRegister {
    RESET = 0x06,
    START = 0x08,
    POWERDOWN = 0x02,
    RDATA = 0x10,
    WREG = 0x40,
};  
class ADSInterface {
    public:
        void init_uart();
        void init();
        void read_current_data();
        void power_up();
        void power_down();
};
