#ifndef ROBIN_SIM_GZ__BEAD_MODEL_HH_
#define ROBIN_SIM_GZ__BEAD_MODEL_HH_

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace robin_sim_gz
{

// ── Nominal process point for 1.0 mm ER70S-6, Ar + CO2 (8%) ────────
constexpr double NOMINAL_CURRENT_A          = 180.0;
constexpr double NOMINAL_WFS_M_PER_MIN      = 7.0;
constexpr double NOMINAL_TRAVEL_SPEED_MM_S  = 6.0;

constexpr double NOMINAL_WIDTH_MM  = 7.0;
constexpr double NOMINAL_HEIGHT_MM = 1.8;

// Scaling exponents
constexpr double WIDTH_HEAT_EXP  = 0.35;
constexpr double WIDTH_DEP_EXP   = 0.20;
constexpr double HEIGHT_HEAT_EXP = 0.10;
constexpr double HEIGHT_DEP_EXP  = 0.75;

// Conservative clamps
constexpr double MIN_WIDTH_MM  = 2.0;
constexpr double MAX_WIDTH_MM  = 15.0;
constexpr double MIN_HEIGHT_MM = 0.4;
constexpr double MAX_HEIGHT_MM = 6.0;

// ── Bead cross-section parameters ───────────────────────────────────
struct BeadSection
{
  double width;              // metres
  double height;             // metres
  double toe_angle_rad;      // atan(4H/W) for parabolic section
};

// ── Contact angle from parabolic cross-section ──────────────────────
inline double solve_contact_angle(double width_m, double height_m)
{
  const double W = std::max(width_m, 1e-12);
  const double H = std::max(height_m, 0.0);
  return std::atan(4.0 * H / W);
}

// ── Bead dimensions from welding parameters ─────────────────────────
inline BeadSection bead_from_params(
  double current_A,
  double wfs_m_per_min,
  double travel_speed_mm_per_s)
{
  const double I   = std::max(current_A, 40.0);
  const double WFS = std::max(wfs_m_per_min, 1.0);
  const double TS  = std::max(travel_speed_mm_per_s, 0.5);

  const double heat_like = (I / NOMINAL_CURRENT_A)
                         * (NOMINAL_TRAVEL_SPEED_MM_S / TS);
  const double dep_like  = (WFS / NOMINAL_WFS_M_PER_MIN)
                         * (NOMINAL_TRAVEL_SPEED_MM_S / TS);

  double w_mm = NOMINAL_WIDTH_MM
              * std::pow(heat_like, WIDTH_HEAT_EXP)
              * std::pow(dep_like,  WIDTH_DEP_EXP);
  double h_mm = NOMINAL_HEIGHT_MM
              * std::pow(heat_like, HEIGHT_HEAT_EXP)
              * std::pow(dep_like,  HEIGHT_DEP_EXP);

  w_mm = std::clamp(w_mm, MIN_WIDTH_MM, MAX_WIDTH_MM);
  h_mm = std::clamp(h_mm, MIN_HEIGHT_MM, MAX_HEIGHT_MM);

  const double w = w_mm / 1000.0;
  const double h = h_mm / 1000.0;
  return {w, h, solve_contact_angle(w, h)};
}

// ── Parabolic cross-section profile ─────────────────────────────────
//
//   z(x) = H * (1 - (2x/W)^2)
//
// Solved for x as a function of z:
//   x(z) = (W/2) * sqrt(1 - z/H)
//
// Returns a closed polygon (x, y) centred at (0, 0) with
// y in [-H/2, +H/2].  Suitable for SDF <polyline> extrusion.
inline std::vector<std::pair<double, double>> bead_cross_section(
    const BeadSection & sec, int n_per_side = 16)
{
  const double a = sec.width / 2.0;
  const double h = sec.height;

  if (h < 1e-9 || a < 1e-9)
    return {{-a, -h / 2.0}, {0.0, h / 2.0}, {a, -h / 2.0}};

  const int n = std::max(n_per_side, 2);

  // Right side from toe (x=a, z=0) to apex (x=0, z=h)
  std::vector<std::pair<double, double>> right;
  right.reserve(n + 1);
  for (int j = 0; j <= n; ++j) {
    double z = h * static_cast<double>(j) / n;
    double x = a * std::sqrt(std::max(0.0, 1.0 - z / h));
    right.push_back({x, z - h / 2.0});
  }

  // Closed polygon: left side (bottom->top) + right side (top->bottom)
  std::vector<std::pair<double, double>> poly;
  poly.reserve(2 * n + 1);
  for (int j = 0; j <= n; ++j)
    poly.push_back({-right[j].first, right[j].second});
  for (int j = n - 1; j >= 0; --j)
    poly.push_back({ right[j].first, right[j].second});
  return poly;
}

}  // namespace robin_sim_gz

#endif  // ROBIN_SIM_GZ__BEAD_MODEL_HH_