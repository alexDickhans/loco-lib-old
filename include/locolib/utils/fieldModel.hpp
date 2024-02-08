#pragma once

#include "locolib/localization/particleFilter/particle.hpp"
#include <limits>
#include "pros/gps.hpp"

namespace Loco {


	class FieldModel {

	public:
		QLength getDistance(const Particle& particle) {
			return std::numeric_limits<double>::quiet_NaN();
		}

        static Eigen::Vector3d fromGps(pros::c::gps_status_s gpsStatus) {
            return Eigen::Vector3d({gpsStatus.y, -gpsStatus.x, -gpsStatus.pitch * degree.Convert(radian)});
        }

		static bool isOnLine() {

		}

		static Eigen::Vector2d minPosition;
		static Eigen::Vector2d maxPosition;
	};

	Eigen::Vector2d FieldModel::minPosition = Eigen::Vector2d(-1.8, -1.8);
	Eigen::Vector2d FieldModel::maxPosition = Eigen::Vector2d(1.8, 1.8);

    Particle::Particle(pros::c::gps_status_s gpsStatus) : Particle(FieldModel::fromGps(gpsStatus)) {
    }
}