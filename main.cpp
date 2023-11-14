#include <vector>
#include "PhotonVirtual.hpp"
#include <algorithm>
int char2int(char input)
{
  if(input >= '0' && input <= '9')
    return input - '0';
  if(input >= 'A' && input <= 'F')
    return input - 'A' + 10;
  if(input >= 'a' && input <= 'f')
    return input - 'a' + 10;
  throw std::invalid_argument("Invalid input string");
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        cout << "Usage: " << argv[0] << " <marlin_port> <photon_port>" << endl;
        return 1;
    }
    PhotonVirtual photonVirtual(argv[1], argv[2]);

    while(true){
        string command;
        if(photonVirtual.newCommand(&command)){
            // if (command.find("M485") != std::string::npos) {
                // "M485 " is found in the command string
            uint8_t buf[64];
            int j=0;
            for(int i=5;i<command.length()-1;i++){
                buf[i-5] = char2int(command[i]);
                printf("%x\t", buf[i-5]);
                j++;
            }
            cout << endl;
            uint8_t buf2[32];
            for(int i=0;i<j;i+=2){
                buf2[i/2] = buf[i] << 4 | buf[i+1]&0xf;
            }
            for(int i=0;i<j/2;i++){
                printf("%x\t", buf2[i]);
            }
            cout << endl;
            cout << endl;
            
            write(photonVirtual.serial_port_photon, buf2, j/2);

            uint8_t read_buf[32];
            memset(&read_buf, '\0', sizeof(read_buf));
            int num_bytes = read(photonVirtual.serial_port_photon, &read_buf, sizeof(read_buf));
            if (num_bytes > 0) {
                printf("Read %i bytes. Received message from feeder: %s\n", num_bytes, read_buf);
                for(int i=0;i<num_bytes;i++){
                    printf("%x\t", read_buf[i]);
                }
                cout<< endl;
                cout<< endl;
                char buf4[64];
                memset(&buf4, '\0', sizeof(buf4));
                string reply("rs485-reply: ");
                // write(photonVirtual.serial_port_marlin, reply.c_str(), reply.size());
                // write(photonVirtual.serial_port_marlin, read_buf, num_bytes);
                // write(photonVirtual.serial_port_marlin, "\n", 1);
                for(int i=0;i<num_bytes*2;i+=2){
                    sprintf(&buf4[i], "%x", (read_buf[i/2]>>4)&0xf); 
                    sprintf(&buf4[i+1], "%x", read_buf[i/2]&0xf); 
                }
                string str(buf4);
                transform(str.begin(), str.end(), str.begin(), ::toupper);
                string reply2 = reply+str+"\r";
                cout << reply2.size() << " " << reply2 << endl;

                write(photonVirtual.serial_port_marlin, reply2.c_str(), reply2.size());
                string reply3("ok\n");
                cout << reply3;
                write(photonVirtual.serial_port_marlin, reply3.c_str(), reply3.size());
                // sleep(2); //required to make flush work, for some reason
                // tcflush(photonVirtual.serial_port_marlin,TCIOFLUSH);
            }else{
                string reply("rs485-reply: TIMEOUT\n");
                cout << reply;
                write(photonVirtual.serial_port_marlin, reply.c_str(), reply.size());
            }
        }
    }
    
    return 0;
}
