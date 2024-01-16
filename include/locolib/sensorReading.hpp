#pragma once

#include "units/units.hpp"
#include "time.hpp"

namespace Loco {
	template <typename T>
	class SensorReading {
	private:
		QTime measurementTime;
		T measurement;
		T std;
	public:
		SensorReading() = delete;

		SensorReading(T measurement, T std) : SensorReading(measurement, std, Time::currentTime()) {}

		SensorReading(T measurement, T std, QTime measurementTime) {
			this->measurementTime = measurementTime;
			this->measurement = measurement;
			this->std = std;
		}

		T getMeasurement() const {
			return measurement;
		}

		T getStd() const {
			return std;
		}

		QTime getTime() const {
			return measurementTime;
		}
	};

	typedef SensorReading<QLength> DistanceReading;
	typedef SensorReading<QSpeed> VelocityReading;
}
