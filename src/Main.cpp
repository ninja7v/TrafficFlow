/////////////////////////////////////////////////////////////////////
// Traffic light modelisation by Luc and Claire PREVOST, 2022-2025 //
/////////////////////////////////////////////////////////////////////

// Libraries
#include <cstring>  // To use strcmp
#include <iostream> // To use std::cout
// Header files
#include "../headers/Network.h"

int main(int argc, char** argv) {
   // INITIALIZATION
   Network net = Network();

   // OPTIONS
   for (int i = 1; i < argc; ++i) {
      if (std::strcmp(argv[i], "--terminal_output") == 0) {
         net.setTerminalOutput(true); // Print the whole control panel in the terminal
      } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
         std::cout << "Usage: " << argv[0] << " [--terminal_output]\n"
                   << "  --terminal_output  Print the whole control panel in the terminal every 5 s\n";
         return 0;
      } else {
         std::cerr << "Unknown option: " << argv[i] << " (see --help)\n";
         return 1;
      }
   }

   // VISUALTISATION
   net.displayNetwork();

   return 0;
}
