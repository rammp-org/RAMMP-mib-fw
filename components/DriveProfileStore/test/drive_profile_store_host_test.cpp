// Host test for DriveProfileStore: loading, fallback to the compiled-in
// defaults, set / reset with and without persist, and files the store must not
// use (corrupt, from newer firmware, or holding invalid limits). Runs against
// the real espp PersistentData and FileSystem host builds in a temp directory.
//
// Build & run from the repo root after an `idf.py build` has fetched
// managed_components/ (one command; -isystem keeps third-party warnings out):
//   M=managed_components; c++ -std=c++23 -Wall -Wextra -Werror
//     -I components/DriveProfileStore/include -I components/DriveController/include
//     -I components/MIBConfig/include
//     -I external/rammp-rtps/components/rammp_rtps_messages/include
//     -isystem $M/espp__base_component/include -isystem $M/espp__logger/include
//     -isystem $M/espp__format/include -isystem $M/espp__format/detail/fmt/include
//     -isystem $M/espp__persistent_data/include -isystem $M/espp__serialization/include
//     -isystem $M/espp__serialization/detail/alpaca/include
//     -isystem $M/espp__file_system/include -isystem $M/espp__trajectory_planner/include
//     -isystem $M/espp__task/include -isystem $M/espp__timer/include -DFMT_HEADER_ONLY
//     components/DriveProfileStore/test/drive_profile_store_host_test.cpp
//     components/DriveProfileStore/src/drive_profile_store.cpp
//     $M/espp__file_system/src/file_system.cpp $M/espp__logger/src/logger.cpp
//     -o /tmp/drive_profile_store_test && /tmp/drive_profile_store_test

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <vector>

#include "drive_profile_store.hpp"

namespace fs = std::filesystem;
using mib::DriveProfileStore;
using mib::config::DriveProfileConfig;
using Profile = MIB::DriveProfile;
using Result = MIB::DriveProfileResult;
using Source = MIB::DriveProfileSource;

static int g_failures = 0;
static int g_checks = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    ++g_checks;                                                                                    \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static bool same(const DriveProfileConfig &a, const DriveProfileConfig &b) {
  return a.max_linear_velocity_mps == b.max_linear_velocity_mps &&
         a.max_reverse_linear_velocity_mps == b.max_reverse_linear_velocity_mps &&
         a.max_angular_velocity_radps == b.max_angular_velocity_radps &&
         a.max_linear_acceleration_mps2 == b.max_linear_acceleration_mps2 &&
         a.max_angular_acceleration_radps2 == b.max_angular_acceleration_radps2 &&
         a.max_linear_jerk_mps3 == b.max_linear_jerk_mps3 &&
         a.max_angular_jerk_radps3 == b.max_angular_jerk_radps3 &&
         a.stopping_linear_acceleration_mps2 == b.stopping_linear_acceleration_mps2 &&
         a.stopping_angular_acceleration_radps2 == b.stopping_angular_acceleration_radps2 &&
         a.max_centripetal_acceleration_mps2 == b.max_centripetal_acceleration_mps2;
}

static std::vector<char> read_bytes(const fs::path &path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), {}};
}

static void write_bytes(const fs::path &path, const std::vector<char> &bytes) {
  std::ofstream(path, std::ios::binary).write(bytes.data(), bytes.size());
}

// A hypothetical next schema version: what a file from newer firmware holds.
struct DriveProfileConfigV2 {
  static constexpr uint32_t VERSION = 2;
  float max_linear_velocity_mps{1.0f};
  float some_new_field{1.0f};
  static DriveProfileConfigV2 migrate(const DriveProfileConfig &v1) {
    return {.max_linear_velocity_mps = v1.max_linear_velocity_mps};
  }
};

int main() {
  const fs::path dir = fs::temp_directory_path() / "drive_profile_store_test";
  fs::remove_all(dir);
  fs::create_directories(dir);
  const fs::path normal_file = dir / "drive_profile_normal.bin";

  DriveProfileConfig tuned = mib::config::drive_profile_normal;
  tuned.max_linear_velocity_mps = 1.2f;
  tuned.max_linear_jerk_mps3 = 5.0f;

  std::printf("no files: every profile is its compiled-in default\n");
  {
    DriveProfileStore store(dir);
    store.load();
    for (auto p : {Profile::LOW, Profile::NORMAL, Profile::HIGH}) {
      CHECK(store.source(p) == Source::DEFAULT);
      CHECK(same(store.get(p), DriveProfileStore::default_values(p)));
    }
    CHECK(fs::is_empty(dir)); // defaults are never written
  }

  std::printf("set with persist saves, and the next boot loads it\n");
  {
    DriveProfileStore store(dir);
    store.load();
    CHECK(store.set(Profile::NORMAL, tuned, true) == Result::OK);
    CHECK(store.source(Profile::NORMAL) == Source::SAVED);
    CHECK(same(store.get(Profile::NORMAL), tuned));
    CHECK(fs::exists(normal_file));
  }
  {
    DriveProfileStore store(dir);
    CHECK(store.source(Profile::NORMAL) == Source::DEFAULT); // nothing read before load()
    store.load();
    CHECK(store.source(Profile::NORMAL) == Source::SAVED);
    CHECK(same(store.get(Profile::NORMAL), tuned));
    CHECK(store.source(Profile::LOW) == Source::DEFAULT);
  }

  std::printf("set without persist lasts until reboot\n");
  {
    DriveProfileStore store(dir);
    store.load();
    DriveProfileConfig slow = mib::config::drive_profile_low;
    slow.max_angular_velocity_radps = 0.5f;
    CHECK(store.set(Profile::LOW, slow, false) == Result::OK);
    CHECK(store.source(Profile::LOW) == Source::UNSAVED);
    CHECK(same(store.get(Profile::LOW), slow));
    CHECK(!fs::exists(dir / "drive_profile_low.bin"));
  }
  {
    DriveProfileStore store(dir);
    store.load();
    CHECK(store.source(Profile::LOW) == Source::DEFAULT);
    CHECK(same(store.get(Profile::LOW), mib::config::drive_profile_low));
  }

  std::printf("invalid limits are rejected whole, nothing changes\n");
  {
    DriveProfileStore store(dir);
    store.load();
    const auto before = read_bytes(normal_file);
    for (float bad : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::infinity()}) {
      DriveProfileConfig v = tuned;
      v.max_linear_velocity_mps = bad;
      CHECK(store.set(Profile::NORMAL, v, true) == Result::INVALID);
    }
    DriveProfileConfig v = tuned;
    v.max_angular_jerk_radps3 = -0.1f; // jerk may be 0, never negative
    CHECK(store.set(Profile::NORMAL, v, true) == Result::INVALID);
    v = tuned;
    v.stopping_linear_acceleration_mps2 = 0.0f; // a stop must be able to stop
    CHECK(store.set(Profile::NORMAL, v, false) == Result::INVALID);
    v = tuned;
    v.max_reverse_linear_velocity_mps = 0.0f;   // 0 = same as forward: allowed
    v.max_centripetal_acceleration_mps2 = 0.0f; // 0 = no limit: allowed
    v.max_linear_jerk_mps3 = 0.0f;              // 0 = trapezoidal: allowed
    CHECK(store.set(Profile::NORMAL, v, false) == Result::OK);
    CHECK(store.source(Profile::NORMAL) == Source::UNSAVED);
    CHECK(read_bytes(normal_file) == before);
  }

  std::printf("an unknown profile is a bad request\n");
  {
    DriveProfileStore store(dir);
    CHECK(store.set(static_cast<Profile>(7), tuned, true) == Result::BAD_REQUEST);
    CHECK(store.reset(static_cast<Profile>(7), true) == Result::BAD_REQUEST);
    CHECK(same(store.get(static_cast<Profile>(7)), mib::config::drive_profile_normal));
  }

  std::printf("a corrupt file falls back to the default and is left in place\n");
  {
    auto bytes = read_bytes(normal_file);
    bytes.back() ^= 0x01;
    write_bytes(normal_file, bytes);
    DriveProfileStore store(dir);
    store.load();
    CHECK(store.source(Profile::NORMAL) == Source::DEFAULT);
    CHECK(same(store.get(Profile::NORMAL), mib::config::drive_profile_normal));
    CHECK(read_bytes(normal_file) == bytes);
    // a persistent set replaces it
    CHECK(store.set(Profile::NORMAL, tuned, true) == Result::OK);
    DriveProfileStore again(dir);
    again.load();
    CHECK(again.source(Profile::NORMAL) == Source::SAVED);
  }

  std::printf("a truncated file falls back to the default\n");
  {
    auto bytes = read_bytes(normal_file);
    const auto good = bytes;
    bytes.resize(bytes.size() / 2);
    write_bytes(normal_file, bytes);
    DriveProfileStore store(dir);
    store.load();
    CHECK(store.source(Profile::NORMAL) == Source::DEFAULT);
    write_bytes(normal_file, good);
  }

  std::printf("a file from newer firmware is refused and left in place\n");
  {
    const fs::path high_file = dir / "drive_profile_high.bin";
    espp::PersistentData<DriveProfileConfigV2, DriveProfileConfig> newer(
        {.file_path = high_file, .auto_load = false});
    std::error_code ec;
    CHECK(newer.save(DriveProfileConfigV2{.max_linear_velocity_mps = 3.0f}, ec));
    const auto bytes = read_bytes(high_file);
    DriveProfileStore store(dir);
    store.load();
    CHECK(store.source(Profile::HIGH) == Source::DEFAULT);
    CHECK(same(store.get(Profile::HIGH), mib::config::drive_profile_high));
    CHECK(read_bytes(high_file) == bytes);
    fs::remove(high_file);
  }

  std::printf("a well-formed file holding invalid limits falls back to the default\n");
  {
    DriveProfileConfig zero = tuned;
    zero.max_angular_velocity_radps = 0.0f;
    std::error_code ec;
    DriveProfileStore::File raw({.file_path = normal_file, .auto_load = false});
    CHECK(raw.save(zero, ec));
    DriveProfileStore store(dir);
    store.load();
    CHECK(store.source(Profile::NORMAL) == Source::DEFAULT);
    CHECK(same(store.get(Profile::NORMAL), mib::config::drive_profile_normal));
    CHECK(raw.save(tuned, ec));
  }

  std::printf("reset without persist lasts until reboot; with persist deletes the file\n");
  {
    DriveProfileStore store(dir);
    store.load();
    CHECK(store.source(Profile::NORMAL) == Source::SAVED);
    CHECK(store.reset(Profile::NORMAL, false) == Result::OK);
    CHECK(store.source(Profile::NORMAL) == Source::UNSAVED); // a reboot loads the file
    CHECK(same(store.get(Profile::NORMAL), mib::config::drive_profile_normal));
    CHECK(store.reset(Profile::LOW, false) == Result::OK);
    CHECK(store.source(Profile::LOW) == Source::DEFAULT); // nothing saved to go back to
  }
  {
    DriveProfileStore store(dir);
    store.load();
    CHECK(same(store.get(Profile::NORMAL), tuned));
    CHECK(store.reset(Profile::NORMAL, true) == Result::OK);
    CHECK(store.source(Profile::NORMAL) == Source::DEFAULT);
    CHECK(!fs::exists(normal_file));
  }
  {
    DriveProfileStore store(dir);
    store.load();
    CHECK(store.source(Profile::NORMAL) == Source::DEFAULT);
    CHECK(same(store.get(Profile::NORMAL), mib::config::drive_profile_normal));
  }

  std::printf("a failed save keeps the new limits in use until reboot\n");
  {
    const fs::path missing = dir / "no_such_dir";
    DriveProfileStore store(missing);
    CHECK(store.set(Profile::NORMAL, tuned, true) == Result::SAVE_FAILED);
    CHECK(store.source(Profile::NORMAL) == Source::UNSAVED);
    CHECK(same(store.get(Profile::NORMAL), tuned));
  }

  fs::remove_all(dir);
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
