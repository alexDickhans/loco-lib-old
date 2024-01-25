#pragma once

#include "timeOfFlight.hpp"
#include "pros/distance.hpp"

#define LIDAR_MAX_DISTANCE 1.3_m

namespace Loco {

	class DistanceSensor : public TimeOfFlight {
		pros::Distance& distance;
	public:
		DistanceSensor() = delete;
		explicit DistanceSensor(pros::Distance& distance, FieldModel& model, QLength maxLength = LIDAR_MAX_DISTANCE) : distance(distance),
																																TimeOfFlight(model, maxLength) {

		}

		QLength getReading() override {
			return distance.get() * 1_mm;
		}

		QLength getStd() override {
			return distance.get_confidence() * 1_mm;
		}
	};
}
