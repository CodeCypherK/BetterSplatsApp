// bs_synth — synthetic capture-session generator.
//
// Renders nothing: it synthesizes a scene (textured planes + scatter points),
// a camera trajectory, projected images with controllable noise, and LiDAR
// depth maps, then writes a session directory in the exact on-disk format the
// iPhone app produces. Ground-truth poses/points are written alongside so
// tests can measure recovery error end-to-end.
//
// The full generator lands in M1 with the session writer; this skeleton
// validates the CLI plumbing and the engine link.

#include <cstdio>
#include <cstring>

#include "bs/bs_api.h"

int main(int argc, char** argv) {
  if (argc >= 2 && std::strcmp(argv[1], "--version") == 0) {
    std::printf("bs_synth %s\n", bs_version());
    return 0;
  }
  if (argc >= 2 && std::strcmp(argv[1], "--selftest") == 0) {
    char buf[512];
    const bs_result r = bs_selftest(buf, sizeof(buf));
    std::printf("%s\n", buf);
    return r == BS_OK ? 0 : 1;
  }
  std::fprintf(stderr,
               "bs_synth: synthetic session generator (M1)\n"
               "usage: bs_synth --version | --selftest\n"
               "       bs_synth <out_session_dir> [--scene room|corridor] "
               "(lands in M1)\n");
  return 2;
}
