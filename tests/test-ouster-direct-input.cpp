/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   test-ouster-direct-input.cpp
 * @brief  Unit tests for OusterDirectInput module registration,
 *         YAML validation, and helper functions.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_input_ouster/OusterDirectInput.h>
#include <mola_kernel/interfaces/ExecutableBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/poses/CPose3D.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

// ---- Test helpers ----------------------------------------------------------

int testsPassed = 0;
int testsFailed = 0;

void check(bool condition, const std::string& name)
{
  if (condition)
  {
    std::cout << "  [PASS] " << name << "\n";
    testsPassed++;
  }
  else
  {
    std::cerr << "  [FAIL] " << name << "\n";
    testsFailed++;
  }
}

// ---- Tests -----------------------------------------------------------------

void testModuleRegistration()
{
  std::cout << "-- Module registration --\n";

  // The MRPT_INITIALIZER in OusterDirectInput.cpp should have registered
  // the class so that ExecutableBase::Factory can find it.
  auto mod = mola::ExecutableBase::Factory("mola::OusterDirectInput");
  check(mod != nullptr, "Factory creates mola::OusterDirectInput");

  if (mod)
  {
    auto rds = std::dynamic_pointer_cast<mola::RawDataSourceBase>(mod);
    check(rds != nullptr, "Instance is a RawDataSourceBase");
  }
}

void testModuleRTTI()
{
  std::cout << "-- RTTI class info --\n";

  auto mod = mola::ExecutableBase::Factory("mola::OusterDirectInput");
  check(mod != nullptr, "Factory returns non-null");

  if (mod)
  {
    const auto* classInfo = mod->GetRuntimeClass();
    check(classInfo != nullptr, "GetRuntimeClass() non-null");

    if (classInfo)
    {
      const std::string name = classInfo->className;
      check(
          name == "mola::OusterDirectInput",
          "className == 'mola::OusterDirectInput', got '" + name + "'");
    }
  }
}

void testYamlValidationMissingParams()
{
  std::cout << "-- YAML validation: missing params --\n";

  auto mod = mola::ExecutableBase::Factory("mola::OusterDirectInput");
  check(mod != nullptr, "Module created for YAML test");

  if (!mod)
  {
    return;
  }

  // No 'params' key at all — should throw
  {
    bool threw = false;
    try
    {
      mrpt::containers::yaml cfg;
      cfg["type"] = "mola::OusterDirectInput";
      // deliberately no "params" key
      mod->initialize(cfg);
    }
    catch (const std::exception&)
    {
      threw = true;
    }
    check(threw, "Throws when 'params' key is missing");
  }

  // Neither sensor_hostname nor pcap_file — should throw
  {
    // Need a fresh module since initialize is final and may have
    // partially modified state
    auto mod2  = mola::ExecutableBase::Factory("mola::OusterDirectInput");
    bool threw = false;
    try
    {
      mrpt::containers::yaml cfg;
      cfg["params"]                       = mrpt::containers::yaml::Map();
      cfg["params"]["lidar_sensor_label"] = "lidar";
      // Neither sensor_hostname nor pcap_file
      mod2->initialize(cfg);
    }
    catch (const std::exception&)
    {
      threw = true;
    }
    check(threw, "Throws when neither sensor_hostname nor pcap_file given");
  }

  // pcap_file without metadata_json — should throw
  {
    auto mod3  = mola::ExecutableBase::Factory("mola::OusterDirectInput");
    bool threw = false;
    try
    {
      mrpt::containers::yaml cfg;
      cfg["params"]              = mrpt::containers::yaml::Map();
      cfg["params"]["pcap_file"] = "/nonexistent/file.pcap";
      // no metadata_json
      mod3->initialize(cfg);
    }
    catch (const std::exception&)
    {
      threw = true;
    }
    check(threw, "Throws when pcap_file given without metadata_json");
  }

  // pcap_file + metadata_json but files don't exist — should throw
  {
    auto mod4  = mola::ExecutableBase::Factory("mola::OusterDirectInput");
    bool threw = false;
    try
    {
      mrpt::containers::yaml cfg;
      cfg["params"]                  = mrpt::containers::yaml::Map();
      cfg["params"]["pcap_file"]     = "/nonexistent/file.pcap";
      cfg["params"]["metadata_json"] = "/nonexistent/meta.json";
      mod4->initialize(cfg);
    }
    catch (const std::exception&)
    {
      threw = true;
    }
    check(threw, "Throws when pcap/metadata files don't exist");
  }
}

void testTimestampConversion()
{
  std::cout << "-- Timestamp conversion --\n";

  // A known Unix timestamp:
  // 2025-01-01 00:00:00 UTC = 1735689600 seconds
  constexpr uint64_t knownEpochSec  = 1735689600ULL;
  constexpr uint64_t knownEpochNsec = knownEpochSec * 1'000'000'000ULL;

  // We can't call the anonymous-NS helper directly, but we can
  // verify that the timestamp round-trip through mrpt::Clock is sane.
  // Build the same conversion manually:
  using namespace std::chrono;
  const auto dur = nanoseconds(knownEpochNsec);
  const auto tp  = std::chrono::time_point<std::chrono::system_clock, nanoseconds>(dur);
  const auto mrptTp =
      mrpt::Clock::time_point(duration_cast<mrpt::Clock::duration>(tp.time_since_epoch()));

  const double mrptDouble = mrpt::Clock::toDouble(mrptTp);

  // Should be very close to the known epoch (within 1 ms tolerance)
  const double error = std::abs(mrptDouble - static_cast<double>(knownEpochSec));
  check(
      error < 0.001,
      "Timestamp 2025-01-01T00:00:00Z round-trips within 1 ms "
      "(error=" +
          std::to_string(error) + "s)");

  // Zero timestamp should produce epoch (time_point at zero)
  const auto   zeroTp     = mrpt::Clock::time_point(mrpt::Clock::duration(0));
  const double zeroDouble = mrpt::Clock::toDouble(zeroTp);
  check(
      std::abs(zeroDouble) < 1.0,
      "Zero nsec maps to near-epoch (got " + std::to_string(zeroDouble) + ")");
}

void testPoseStringParsing()
{
  std::cout << "-- Pose string parsing --\n";

  // We test the same logic that parsePoseString() uses, which is
  // CPose3D::FromXYZYawPitchRoll. This validates the convention.
  {
    const auto p = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        1.0, 2.0, 3.0, mrpt::DEG2RAD(90.0), mrpt::DEG2RAD(0.0), mrpt::DEG2RAD(0.0));

    check(std::abs(p.x() - 1.0) < 1e-9, "Pose x=1.0");
    check(std::abs(p.y() - 2.0) < 1e-9, "Pose y=2.0");
    check(std::abs(p.z() - 3.0) < 1e-9, "Pose z=3.0");

    double yaw = 0, pitch = 0, roll = 0;
    p.getYawPitchRoll(yaw, pitch, roll);
    check(std::abs(yaw - mrpt::DEG2RAD(90.0)) < 1e-9, "Pose yaw=90 deg");
  }

  // Identity pose
  {
    const auto p = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0, 0, 0);
    check(
        std::abs(p.x()) < 1e-9 && std::abs(p.y()) < 1e-9 && std::abs(p.z()) < 1e-9,
        "Identity pose is at origin");
  }
}

}  // namespace

int main()
{
  try
  {
    std::cout << "=== mola_input_ouster unit tests ===\n";

    testModuleRegistration();
    testModuleRTTI();
    testYamlValidationMissingParams();
    testTimestampConversion();
    testPoseStringParsing();

    std::cout << "\n=== Results: " << testsPassed << " passed, " << testsFailed << " failed ===\n";

    return testsFailed > 0 ? 1 : 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Unhandled exception:\n" << e.what() << "\n";
    return 1;
  }
}
