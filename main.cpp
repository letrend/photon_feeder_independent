#include <vector>
#include "PhotonVirtual.hpp"

int main(int argc, char *argv[]) {
    if (argc != 3) {
        cout << "Usage: " << argv[0] << " <marlin_port> <photon_port>" << endl;
        return 1;
    }
    PhotonVirtual photonVirtual(argv[1], argv[2]);
    photonVirtual.run();
    
    return 0;
}
