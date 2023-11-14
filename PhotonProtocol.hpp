#include <cstdint>
#include <cstring>
#include <iostream>

#define UUID_LENGTH 12
#define VENDOR_SPECIFIC_OPTIONS_LENGTH 20  // Chosen by fair d20 roll

#define PHOTON_NETWORK_CONTROLLER_ADDRESS 0x00
#define PHOTON_NETWORK_BROADCAST_ADDRESS 0xFF

#define PACKED __attribute__ ((packed))

struct PACKED PhotonPacketHeader {
    uint8_t toAddress;
    uint8_t fromAddress;
    uint8_t packetId;
    uint8_t payloadLength;
    uint8_t crc;
};


struct PACKED MoveCommand {
    uint8_t distance;
};

struct PACKED GetFeederAddressCommand {
    uint8_t uuid[UUID_LENGTH];
};

struct PACKED InitializeFeederCommand {
    uint8_t uuid[UUID_LENGTH];
};

struct PACKED VendorOptionsCommand {
    uint8_t options[VENDOR_SPECIFIC_OPTIONS_LENGTH];
};

struct PACKED ProgramFeederFloorAddressCommand {
    uint8_t uuid[UUID_LENGTH];
    uint8_t address;
};

struct PACKED IdentifyFeederCommand {
    uint8_t uuid[UUID_LENGTH];
};

struct PACKED PhotonCommand {
    PhotonPacketHeader header;
    uint8_t commandId;
    union {
        MoveCommand move;
        GetFeederAddressCommand getFeederAddress;
        InitializeFeederCommand initializeFeeder;
        VendorOptionsCommand vendorOptions;
        ProgramFeederFloorAddressCommand programFeederFloorAddress;
        IdentifyFeederCommand identifyFeeder;
    };
};

class CRC8_107{
    public:
        void add(uint8_t data){
            crc ^= (data << 8);
            for (size_t bit_n = 0; bit_n < 8; bit_n++) {
                if (crc & 0x8000)
                    crc ^= (0x1070 << 3);
                crc <<= 1;
            }
        }
        uint8_t getChecksum(){
            return (uint8_t)(crc >> 8);
        }
    private:
        uint32_t crc = 0x0;
};