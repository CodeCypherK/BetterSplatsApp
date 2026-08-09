// bs_replay — run the engine against a captured session directory on Linux.
//
// This is the primary debugging path for the project: the iPhone app exports
// a full-session zip (RAW layer), and this CLI feeds those frames through the
// same engine binary logic that runs on the phone — live pipeline first, then
// the final solve. No Mac or device required to reproduce and fix engine
// behavior.
//
// The frame-feeding loop lands in M1 with the session reader; this skeleton
// validates CLI plumbing and the engine link.

#include <cstdio>
#include <cstring>

#include "bs/bs_api.h"

int main(int argc, char** argv) {
  if (argc >= 2 && std::strcmp(argv[1], "--version") == 0) {
    std::printf("bs_replay %s\n", bs_version());
    return 0;
  }
  if (argc >= 2 && std::strcmp(argv[1], "--selftest") == 0) {
    char buf[512];
    const bs_result r = bs_selftest(buf, sizeof(buf));
    std::printf("%s\n", buf);
    return r == BS_OK ? 0 : 1;
  }
  std::fprintf(stderr,
               "bs_replay: session replay driver (M1)\n"
               "usage: bs_replay --version | --selftest\n"
               "       bs_replay <session_dir> [--live] [--final quality|fast] "
               "(lands in M1+)\n");
  return 2;
}
