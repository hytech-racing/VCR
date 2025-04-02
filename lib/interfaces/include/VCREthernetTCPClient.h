    #ifndef VCR_ETHERNET_TCP_CLIENT_H
    #define VCR_ETHERNET_TCP_CLIENT_H

    #include <Arduino.h>
    #include <QNEthernet.h>
    #include "etl/singleton.h"
    #include "EthernetAddressDefs.h"

    using namespace qindesign::network;

    class VCREthernetTCPClient {
        public:
            VCREthernetTCPClient();
            void begin();
            void sendAndReceive();
            
        private:
            EthernetClient client;
    };

    #endif /* VCR_ETHERNET_TCP_CLIENT_H */