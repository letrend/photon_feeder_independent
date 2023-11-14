#pragma once

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include "PhotonProtocol.hpp"

using namespace std;

class PhotonVirtual {
    public:
        PhotonVirtual(string marlin_port, string photon_port);
        ~PhotonVirtual();
        bool newCommand(string &command);
    private:
        int serial_port_marlin, serial_port_photon;
        char read_buf [256];
};


