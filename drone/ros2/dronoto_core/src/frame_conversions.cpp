#include "dronoto_core/frame_conversions.hpp"

#include <cmath>

namespace dronoto_core
{
namespace frames
{
namespace
{

/// Rotation de pi autour de l'axe (1, 1, 0)/sqrt(2) : envoie NED sur ENU.
/// Constructeur Eigen::Quaterniond(w, x, y, z).
const Eigen::Quaterniond kNedEnu(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);

/// Rotation de pi autour de l'axe X : envoie FRD (aeronautique) sur FLU (base_link).
const Eigen::Quaterniond kFrdFlu(0.0, 1.0, 0.0, 0.0);

}  // namespace

Eigen::Vector3d nedToEnu(const Eigen::Vector3d & ned)
{
  return Eigen::Vector3d(ned.y(), ned.x(), -ned.z());
}

Eigen::Vector3d enuToNed(const Eigen::Vector3d & enu)
{
  // Involution : exactement la meme permutation.
  return Eigen::Vector3d(enu.y(), enu.x(), -enu.z());
}

Eigen::Quaterniond nedFrdToEnuFlu(const Eigen::Quaterniond & q_ned_frd)
{
  return (kNedEnu * q_ned_frd * kFrdFlu).normalized();
}

Eigen::Quaterniond enuFluToNedFrd(const Eigen::Quaterniond & q_enu_flu)
{
  // kNedEnu et kFrdFlu sont leurs propres inverses (rotations de pi),
  // donc la transformation inverse a la meme forme.
  return (kNedEnu * q_enu_flu * kFrdFlu).normalized();
}

double wrapPi(double angle)
{
  // std::remainder ramene dans [-pi, pi]. On envoie -pi sur +pi pour obtenir
  // l'intervalle semi-ouvert (-pi, pi] documente dans l'en-tete.
  const double wrapped = std::remainder(angle, 2.0 * M_PI);
  return (wrapped <= -M_PI) ? M_PI : wrapped;
}

double yawNedToEnu(double yaw_ned)
{
  return wrapPi(M_PI / 2.0 - yaw_ned);
}

double yawEnuToNed(double yaw_enu)
{
  return wrapPi(M_PI / 2.0 - yaw_enu);
}

}  // namespace frames
}  // namespace dronoto_core
