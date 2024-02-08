#pragma once

#include "orientationSource.hpp"
#include "pros/gps.hpp"

namespace Loco {
	class GpsOrientationSource : public OrientationSource {
	private:
		pros::Gps& gps;
	public:
		explicit GpsOrientationSource(pros::Gps& gps) : gps(gps) {
		}

		Angle getAngle() override {
			return -this->gps.get_rotation() * 1_deg;
		}
	};
}