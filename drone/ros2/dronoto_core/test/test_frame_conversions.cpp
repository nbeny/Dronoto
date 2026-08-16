#include <gtest/gtest.h>

#include <cmath>

#include "dronoto_core/frame_conversions.hpp"

using dronoto_core::frames::enuFluToNedFrd;
using dronoto_core::frames::enuToNed;
using dronoto_core::frames::nedFrdToEnuFlu;
using dronoto_core::frames::nedToEnu;
using dronoto_core::frames::wrapPi;
using dronoto_core::frames::yawEnuToNed;
using dronoto_core::frames::yawNedToEnu;

constexpr double kEps = 1e-9;

// ---------------------------------------------------------------- positions

TEST(FrameConversions, NedToEnuSwapsXyAndNegatesZ)
{
  // 1 m Nord, 2 m Est, 3 m vers le BAS
  const Eigen::Vector3d ned(1.0, 2.0, 3.0);
  const Eigen::Vector3d enu = nedToEnu(ned);
  EXPECT_NEAR(enu.x(), 2.0, kEps);   // Est
  EXPECT_NEAR(enu.y(), 1.0, kEps);   // Nord
  EXPECT_NEAR(enu.z(), -3.0, kEps);  // Haut : 3 m vers le bas = -3 m vers le haut
}

TEST(FrameConversions, PositionConversionIsAnInvolution)
{
  const Eigen::Vector3d ned(-4.5, 17.25, -0.75);
  const Eigen::Vector3d back = enuToNed(nedToEnu(ned));
  EXPECT_NEAR(back.x(), ned.x(), kEps);
  EXPECT_NEAR(back.y(), ned.y(), kEps);
  EXPECT_NEAR(back.z(), ned.z(), kEps);
}

TEST(FrameConversions, AltitudeSignIsCorrect)
{
  // 50 m d'ALTITUDE = -50 en NED
  const Eigen::Vector3d ned(0.0, 0.0, -50.0);
  EXPECT_NEAR(nedToEnu(ned).z(), 50.0, kEps);
}

// ------------------------------------------------------------- orientations

TEST(FrameConversions, IdentityNedFrdPointsNorthWhichIsPlusYInEnu)
{
  // Identite en NED/FRD = nez au Nord, a plat.
  // En ENU, le Nord est +Y, donc l'axe avant du corps doit pointer vers +Y.
  const Eigen::Quaterniond q_ned = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond q_enu = nedFrdToEnuFlu(q_ned);

  const Eigen::Vector3d forward_enu = q_enu * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(forward_enu.x(), 0.0, 1e-6);
  EXPECT_NEAR(forward_enu.y(), 1.0, 1e-6);
  EXPECT_NEAR(forward_enu.z(), 0.0, 1e-6);
}

TEST(FrameConversions, UpAxisIsPreservedThroughOrientationConversion)
{
  // Corps a plat : l'axe Z du corps pointe vers le BAS en FRD,
  // donc vers le HAUT en FLU apres conversion.
  const Eigen::Quaterniond q_enu = nedFrdToEnuFlu(Eigen::Quaterniond::Identity());
  const Eigen::Vector3d up_enu = q_enu * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(up_enu.z(), 1.0, 1e-6);
}

TEST(FrameConversions, OrientationConversionRoundTrips)
{
  // Attitude quelconque : roulis 0,3 / tangage -0,2 / lacet 1,1 rad
  const Eigen::Quaterniond q_ned =
    Eigen::Quaterniond(Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()));

  const Eigen::Quaterniond back = enuFluToNedFrd(nedFrdToEnuFlu(q_ned));
  // Comparer par produit scalaire : q et -q representent la meme rotation
  EXPECT_NEAR(std::abs(back.dot(q_ned)), 1.0, 1e-9);
}

TEST(FrameConversions, OrientationConversionPreservesNormalization)
{
  const Eigen::Quaterniond q_ned =
    Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()));
  EXPECT_NEAR(nedFrdToEnuFlu(q_ned).norm(), 1.0, 1e-9);
}

// ---------------------------------------------------------------------- cap

TEST(FrameConversions, YawNorthNedIsNinetyDegreesEnu)
{
  EXPECT_NEAR(yawNedToEnu(0.0), M_PI / 2.0, kEps);    // Nord  -> +90 deg ENU
  EXPECT_NEAR(yawNedToEnu(M_PI / 2.0), 0.0, kEps);    // Est   ->   0 deg ENU
  EXPECT_NEAR(yawNedToEnu(-M_PI / 2.0), M_PI, kEps);  // Ouest -> 180 deg ENU
}

TEST(FrameConversions, YawConversionIsAnInvolution)
{
  for (double y = -3.0; y < 3.0; y += 0.37) {
    EXPECT_NEAR(yawEnuToNed(yawNedToEnu(y)), wrapPi(y), 1e-9) << "yaw=" << y;
  }
}

TEST(FrameConversions, WrapPiNormalizesToHalfOpenInterval)
{
  EXPECT_NEAR(wrapPi(0.0), 0.0, kEps);
  EXPECT_NEAR(wrapPi(M_PI), M_PI, kEps);         // borne incluse
  EXPECT_NEAR(wrapPi(-M_PI), M_PI, kEps);        // -pi se ramene a +pi
  EXPECT_NEAR(wrapPi(3.0 * M_PI), M_PI, kEps);
  EXPECT_NEAR(wrapPi(2.0 * M_PI + 0.5), 0.5, kEps);
  EXPECT_NEAR(wrapPi(-2.0 * M_PI - 0.5), -0.5, kEps);
}
