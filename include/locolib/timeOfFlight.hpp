#pragma once

#include "units/units.hpp"
#include "time.hpp"
#include "sensorReading.hpp"

namespace Loco {

	class TimeOfFlight {
	public:
		virtual void update() {}

		virtual DistanceReading getDistance() const { return {0.0, 0.0}; }
		virtual VelocityReading getVelocity() const { return {0.0, 0.0}; }
	};
}
