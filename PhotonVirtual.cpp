
#include "PhotonVirtual.hpp"

// Constructor
PhotonVirtual::PhotonVirtual(string marlin_port, string photon_port) {
    serial_port_marlin = open(marlin_port.c_str(), O_RDWR);
    serial_port_photon = open(photon_port.c_str(), O_RDWR);

    {
        // Create new termios struct, we call it 'tty' for convention
        struct termios tty;

        // Read in existing settings, and handle any error
        if(tcgetattr(serial_port_marlin, &tty) != 0) {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return;
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

        tty.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        // Set in/out baud rate to be 9600
        cfsetispeed(&tty, B2000000);
        cfsetospeed(&tty, B2000000);

        // Save tty settings, also checking for error
        if (tcsetattr(serial_port_marlin, TCSANOW, &tty) != 0) {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return;
        }
    }

    {
        // Create new termios struct, we call it 'tty' for convention
        struct termios tty;

        // Read in existing settings, and handle any error
        if(tcgetattr(serial_port_photon, &tty) != 0) {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return;
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

        tty.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        // Set in/out baud rate to be 9600
        cfsetispeed(&tty, B57600);
        cfsetospeed(&tty, B57600);

        // Save tty settings, also checking for error
        if (tcsetattr(serial_port_photon, TCSANOW, &tty) != 0) {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return;
        }
    }
}

// Destructor
PhotonVirtual::~PhotonVirtual() {
    close(serial_port_marlin);
    close(serial_port_photon);
}

int PhotonVirtual::char2int(char input)
{
  if(input >= '0' && input <= '9')
    return input - '0';
  if(input >= 'A' && input <= 'F')
    return input - 'A' + 10;
  if(input >= 'a' && input <= 'f')
    return input - 'a' + 10;
  throw std::invalid_argument("Invalid input string");
}

void PhotonVirtual::run(){
    string ok("ok\n");
    while(true){
        memset(&read_buf, '\0', sizeof(read_buf));
        int num_bytes = read(serial_port_marlin, &read_buf, sizeof(read_buf));
        if (num_bytes > 0) {
            printf("Received message from openPnP: %s", read_buf);
            string command = string(read_buf);
            
            write(serial_port_marlin, ok.c_str(), ok.size());
            
            uint8_t buf[64];
            int j=0;
            for(int i=5;i<command.length()-1;i++){
                buf[i-5] = char2int(command[i]);
                j++;
            }
            uint8_t buf2[32];
            for(int i=0;i<j;i+=2){
                buf2[i/2] = buf[i] << 4 | buf[i+1]&0xf;
            }
            write(serial_port_photon, buf2, j/2);

            uint8_t read_buf[32];
            memset(&read_buf, '\0', sizeof(read_buf));
            int num_bytes = read(serial_port_photon, &read_buf, sizeof(read_buf));
            if (num_bytes > 0) {
                char buf4[64];
                memset(&buf4, '\0', sizeof(buf4));
                string reply("rs485-reply: ");
                for(int i=0;i<num_bytes*2;i+=2){
                    sprintf(&buf4[i], "%x", (read_buf[i/2]>>4)&0xf); 
                    sprintf(&buf4[i+1], "%x", read_buf[i/2]&0xf); 
                }
                string str(buf4);
                transform(str.begin(), str.end(), str.begin(), ::toupper);
                string reply2 = reply+str+"\r";
                cout << reply2 << endl;
                write(serial_port_marlin, reply2.c_str(), reply2.size());
                string reply3("ok\n");
                write(serial_port_marlin, reply3.c_str(), reply3.size());
            }else{
                string reply("rs485-reply: TIMEOUT\n");
                cout << reply;
                write(serial_port_marlin, reply.c_str(), reply.size());
            }
        }
    }
}