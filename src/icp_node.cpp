
// ICP
#include "icp_cpp/scanmatcher_driver.h"
#include "icp_cpp/program_options.h"

/*
* Program main. Instantiate the scan match driver and parse args.
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_matcher");
  ProgramOptions opts = ParseArgs(argc, argv);
  ScanMatchDriver node(opts);
  return 0;
}
