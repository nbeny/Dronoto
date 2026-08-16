#ifndef DRONOTO_CORE__FRAME_CONVERSIONS_HPP_
#define DRONOTO_CORE__FRAME_CONVERSIONS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dronoto_core
{

/// Conversions entre les conventions PX4 (NED monde / FRD corps) et
/// ROS (ENU monde / FLU corps), conformement a REP-103.
///
/// NED->ENU et ENU->NED sont la MEME operation (permutation x/y, negation z) :
/// c'est une involution. Les deux noms existent pour la lisibilite du site d'appel.
namespace frames
{

/// Position ou vecteur : (x, y, z)_NED -> (y, x, -z)_ENU
Eigen::Vector3d nedToEnu(const Eigen::Vector3d & ned);

/// Position ou vecteur : (x, y, z)_ENU -> (y, x, -z)_NED
Eigen::Vector3d enuToNed(const Eigen::Vector3d & enu);

/// Orientation complete : q_(NED->FRD) -> q_(ENU->FLU)
Eigen::Quaterniond nedFrdToEnuFlu(const Eigen::Quaterniond & q_ned_frd);

/// Orientation complete : q_(ENU->FLU) -> q_(NED->FRD)
Eigen::Quaterniond enuFluToNedFrd(const Eigen::Quaterniond & q_enu_flu);

/// Cap NED (0 = Nord, sens horaire) -> cap ENU (0 = Est, sens trigonometrique).
/// Resultat normalise dans (-pi, pi].
double yawNedToEnu(double yaw_ned);

/// Cap ENU -> cap NED. Resultat normalise dans (-pi, pi].
double yawEnuToNed(double yaw_enu);

/// Ramene un angle dans (-pi, pi].
double wrapPi(double angle);

}  // namespace frames
}  // namespace dronoto_core

#endif  // DRONOTO_CORE__FRAME_CONVERSIONS_HPP_
