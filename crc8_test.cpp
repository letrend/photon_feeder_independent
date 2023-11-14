#include <cstdint>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
// #include "serial_port.h"

using namespace std;

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

int main(){
    // uint8_t data = 0x0a;
    // CRC8_107 checksum;
    // checksum.add(data);
    // uint8_t crc = checksum.getChecksum();
    // char str[10];
    // sprintf(str,"%x %d", crc, crc);
    // cout << str << endl;

    int serial_port = open("/dev/ttyUSB0", O_RDWR);

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B57600);
    cfsetospeed(&tty, B57600);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    PhotonCommand command;
    command.header.toAddress = PHOTON_NETWORK_BROADCAST_ADDRESS;
    command.header.fromAddress = 0x00;
    command.header.packetId = 0xFF;
    command.header.payloadLength = 1;
    command.commandId = 0xc3;
    CRC8_107 checksum;
    checksum.add(command.header.toAddress);
    checksum.add(command.header.fromAddress);
    checksum.add(command.header.packetId);
    checksum.add(command.header.payloadLength);
    checksum.add(command.commandId);
    command.header.crc = checksum.getChecksum();
    uint8_t data[32] = {0};
    memcpy(data, &command, sizeof(PhotonPacketHeader)+1);
    printf("sizeof(PhotonCommand) = %d\n", sizeof(PhotonPacketHeader)+1);
    printf("checksum = %x\n", command.header.crc);
    for(int i = 0; i < sizeof(PhotonPacketHeader)+1; i++){
        printf("%x\t", data[i]);
        if(i % 8 == 7){
            cout << endl;
        }
    }
    cout << endl;

    write(serial_port, data, sizeof(PhotonPacketHeader)+1);
    uint8_t data_received[32] = {0};
    read(serial_port,data_received,18);
    for(int i = 0; i < 18; i++){
        printf("%x\t", data_received[i]);
        if(i % 8 == 7){
            cout << endl;
        }
    }
    cout << endl;
    PhotonCommand command_received;
    memcpy(&command_received, data_received, 18);
    printf("toAddres: %x\n", command_received.header.toAddress);
    printf("fromAddres: %x\n", command_received.header.fromAddress);
    printf("packetId: %x\n", command_received.header.packetId);
    printf("payloadLength: %x\n", command_received.header.payloadLength);
    printf("crc: %x\n", command_received.header.crc);
    printf("payload:\n");
    for(int i = 0; i < command_received.header.payloadLength; i++){
        printf("%x\t", data_received[i+6]);
    }
    cout << endl;

    PhotonCommand command2;
    command2.header.toAddress = command_received.header.fromAddress;
    command2.header.fromAddress = 0x00;
    command2.header.packetId = 0xFF;
    command2.header.payloadLength = 13;
    command2.commandId = 0xc1;
    for(int i = 0; i < UUID_LENGTH; i++){
        command2.identifyFeeder.uuid[i] = data_received[i+6];
    }
    checksum = CRC8_107();
    checksum.add(command2.header.toAddress);
    checksum.add(command2.header.fromAddress);
    checksum.add(command2.header.packetId);
    checksum.add(command2.header.payloadLength);
    checksum.add(command2.commandId);
    for(int i = 0; i < UUID_LENGTH; i++){
        checksum.add(command2.identifyFeeder.uuid[i]);
    }
    command2.header.crc = checksum.getChecksum();
    memcpy(data, &command2, 19);
    write(serial_port, data, 19);
    read(serial_port, data_received,6);
    memcpy(&command_received, data_received, 6);
    printf("toAddres: %x\n", command_received.header.toAddress);
    printf("fromAddres: %x\n", command_received.header.fromAddress);
    printf("packetId: %x\n", command_received.header.packetId);
    printf("payloadLength: %x\n", command_received.header.payloadLength);
    printf("crc: %x\n", command_received.header.crc);
    printf("payload:\n");
    for(int i = 0; i < command_received.header.payloadLength; i++){
        printf("%x\t", data_received[i+6]);
    }
    cout << endl;

    return 0;
}