#pragma once

#include "timeOfFlight.hpp"
#include "pros/distance.hpp"

namespace Loco {
	class DistanceSensor : public TimeOfFlight {
		pros::Distance& distance;
		DistanceReading lastDistanceReading;
		VelocityReading lastVelocityReading;
	public:
		DistanceSensor() = delete;
		explicit DistanceSensor(pros::Distance& distance) : distance(distance), lastDistanceReading(0.0, 0.0, -1.0),
															lastVelocityReading(0.0, 0.0, -1.0) {

		}

		void update() final {
			QLength currentDistance = distance.get() * 1_mm;
			int confidence = distance.get_confidence();
			QSpeed objectVelocity = distance.get_object_velocity() * metre/second;

			lastDistanceReading = DistanceReading(currentDistance, currentDistance/50.0);
			lastVelocityReading = VelocityReading(objectVelocity, objectVelocity/10.0);
		}

		DistanceReading getDistance() const final {
			return lastDistanceReading;
		}

		VelocityReading getVelocity() const final  {
			return lastVelocityReading;
		}
	};
}
