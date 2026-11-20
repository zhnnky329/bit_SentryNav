// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0
//
// Offline command-line probe.  Loads the two maps and a single sweep stored as
// a PCD file, runs the same engine as the node, and prints the outcome.  This
// is the tool used to validate the behaviour against recorded scenes.

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "relocalizer/surface_atlas.hpp"
#include "relocalizer/engine.hpp"

namespace
{

struct Options
{
  std::string plan;
  std::string surface;
  std::string sweep;
  double reference_x{0.0};
  double reference_y{0.0};
  double reference_heading{std::numeric_limits<double>::quiet_NaN()};
  bool has_reference{false};
  int threads{0};
  double coarse_step{-1.0};
  double coarse_floor{-1.0};
  double fine_floor{-1.0};
  double heading_step{-1.0};
  double affinity_sigma{-1.0};
  int keep_hypotheses{-1};
  bool verbose{false};
};

void usage(const char * program)
{
  std::cout <<
    "usage: " << program << " --plan MAP.yaml --surface MAP.pcd --sweep SWEEP.pcd\n"
    "       [--reference X,Y,HEADING_DEG] [--threads N] [--verbose]\n"
    "       [--coarse-step M] [--coarse-floor R] [--fine-floor R]\n"
    "       [--heading-step DEG] [--affinity-sigma M] [--keep N]\n";
}

bool parse_double(const std::string & text, double & value)
{
  try {
    std::size_t used = 0;
    value = std::stod(text, &used);
    return used == text.size();
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_reference(const std::string & text, Options & options)
{
  std::stringstream stream(text);
  std::string token;
  std::vector<double> values;
  while (std::getline(stream, token, ',')) {
    double value = 0.0;
    if (!parse_double(token, value)) {
      return false;
    }
    values.push_back(value);
  }
  if (values.size() != 3) {
    return false;
  }
  options.reference_x = values[0];
  options.reference_y = values[1];
  options.reference_heading = values[2];
  options.has_reference = true;
  return true;
}

bool parse_options(int argc, char ** argv, Options & options)
{
  for (int index = 1; index < argc; ++index) {
    const std::string key = argv[index];
    const bool has_value = index + 1 < argc;
    if (key == "--plan" && has_value) {
      options.plan = argv[++index];
    } else if (key == "--surface" && has_value) {
      options.surface = argv[++index];
    } else if (key == "--sweep" && has_value) {
      options.sweep = argv[++index];
    } else if (key == "--reference" && has_value) {
      if (!parse_reference(argv[++index], options)) {
        return false;
      }
    } else if (key == "--threads" && has_value) {
      options.threads = std::atoi(argv[++index]);
    } else if (key == "--coarse-step" && has_value) {
      options.coarse_step = std::atof(argv[++index]);
    } else if (key == "--coarse-floor" && has_value) {
      options.coarse_floor = std::atof(argv[++index]);
    } else if (key == "--fine-floor" && has_value) {
      options.fine_floor = std::atof(argv[++index]);
    } else if (key == "--heading-step" && has_value) {
      options.heading_step = std::atof(argv[++index]);
    } else if (key == "--affinity-sigma" && has_value) {
      options.affinity_sigma = std::atof(argv[++index]);
    } else if (key == "--keep" && has_value) {
      options.keep_hypotheses = std::atoi(argv[++index]);
    } else if (key == "--verbose") {
      options.verbose = true;
    } else {
      return false;
    }
  }
  return !options.plan.empty() && !options.surface.empty() && !options.sweep.empty();
}

std::string fixed(double value, int digits)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(digits) << value;
  return stream.str();
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  if (!parse_options(argc, argv, options)) {
    usage(argv[0]);
    return 1;
  }

  relocalizer::EngineSettings settings;
  settings.plan_descriptor = options.plan;
  settings.surface_map = options.surface;
  if (options.affinity_sigma > 0.0) {
    settings.affinity_sigma = options.affinity_sigma;
  }
  if (options.threads > 0) {
    settings.search.threads = options.threads;
  }
  if (options.coarse_step > 0.0) {
    settings.search.coarse_step = options.coarse_step;
  }
  if (options.coarse_floor > 0.0) {
    settings.search.coarse_floor = options.coarse_floor;
  }
  if (options.fine_floor > 0.0) {
    settings.search.fine_floor = options.fine_floor;
  }
  if (options.heading_step > 0.0) {
    settings.search.heading_step_deg = options.heading_step;
  }
  if (options.keep_hypotheses > 0) {
    settings.search.keep_hypotheses = options.keep_hypotheses;
  }

  relocalizer::Engine engine;
  std::string error;
  const auto load_start = std::chrono::steady_clock::now();
  if (!engine.configure(settings, error)) {
    std::cerr << "configuration failed: " << error << "\n";
    return 1;
  }
  const double load_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - load_start).count();

  std::vector<Eigen::Vector3d> sweep;
  if (!relocalizer::SurfaceAtlas::read_points(options.sweep, sweep, error)) {
    std::cerr << "cannot read sweep: " << error << "\n";
    return 1;
  }

  std::cout << "plan: " << engine.plan().columns() << "x" << engine.plan().rows()
            << " cells, " << engine.plan().blocked().size() << " total, "
            << fixed(engine.plan().cell_span(), 3) << " m/cell\n";
  std::cout << "surface map: " << engine.atlas().point_count() << " points, "
            << engine.atlas().tier_count() << " voxel tiers\n";
  std::cout << "sweep: " << sweep.size() << " points\n";
  std::cout << "load: " << fixed(load_ms, 1) << " ms\n";

  const auto began = std::chrono::steady_clock::now();
  const relocalizer::Outcome outcome = engine.relocalize(sweep);
  const double elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - began).count();

  std::cout << "search: " << fixed(elapsed, 1) << " ms, "
            << outcome.proposals.size() << " proposal(s)\n";
  for (std::size_t index = 0; index < outcome.proposals.size(); ++index) {
    const relocalizer::Hypothesis & proposal = outcome.proposals[index];
    std::cout << "  #" << index
              << " x=" << fixed(proposal.pose.x, 3)
              << " y=" << fixed(proposal.pose.y, 3)
              << " h=" << fixed(relocalizer::rad_to_deg(proposal.pose.heading), 1)
              << " plan=" << fixed(proposal.plan_agreement, 3)
              << " fit=" << fixed(proposal.coherence, 4)
              << " bad=" << fixed(proposal.mean_penalty, 2)
              << " overlap=" << fixed(proposal.overlap, 3) << "\n";
  }

  std::cout << "confirmed: " << (outcome.confirmed ? "yes" : "no")
            << " (" << outcome.explanation << ")\n";
  std::cout << "pose: x=" << fixed(outcome.pose.x, 3)
            << " y=" << fixed(outcome.pose.y, 3)
            << " heading=" << fixed(relocalizer::rad_to_deg(outcome.pose.heading), 2)
            << " confidence=" << fixed(outcome.confidence, 3) << "\n";
  std::cout << "quality: surfaces=" << fixed(outcome.surface_ratio, 3)
            << " median=" << fixed(outcome.median_distance, 3)
            << " open=" << fixed(outcome.open_space_ratio, 3) << "\n";
  std::cout << "coherence: " << fixed(outcome.coherence, 4)
            << " overlap=" << fixed(outcome.overlap, 3) << "\n";

  if (options.has_reference) {
    const double error_x = outcome.pose.x - options.reference_x;
    const double error_y = outcome.pose.y - options.reference_y;
    const double turn = relocalizer::rad_to_deg(
      relocalizer::heading_gap(
        outcome.pose.heading, relocalizer::deg_to_rad(options.reference_heading)));
    std::cout << "reference error: dx=" << fixed(error_x, 3)
              << " dy=" << fixed(error_y, 3)
              << " distance=" << fixed(std::hypot(error_x, error_y), 3)
              << " heading=" << fixed(turn, 2) << " deg\n";
  }

  return outcome.confirmed ? 0 : 2;
}
