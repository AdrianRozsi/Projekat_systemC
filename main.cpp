#include <systemc>
#include "AudioFile.h"
#include "vp.hpp"
#include <chrono>

using namespace sc_core;
using namespace tlm;
using namespace std;
using namespace std::chrono;

int sc_main(int argc, char* argv[])
{
    if (argc < 3) {
        cout << "Usage: ./program <input.wav> <bass|mid|treble>\n";
        return 1;
    }

    const string wavFile = argv[1];
    const string presetStr = argv[2];
    uint8_t preset;

    if      (presetStr == "bass")   preset = BASS;
    else if (presetStr == "mid")    preset = MID;
    else if (presetStr == "treble") preset = TREBLE;
    else {
        cout << "Preset must be bass | mid | treble\n";
        return 1;
    }

    AudioFile<double> probe;
    
    if (!probe.load(wavFile)) {
        cerr << "[main] ERROR: cannot open input WAV file: " << wavFile << "\n";
        return 1;
    }

    Vp vp("VP", const_cast<char*>(wavFile.c_str()), const_cast<char*>(wavFile.c_str()), preset);
    sc_start();
    return 0;
}
