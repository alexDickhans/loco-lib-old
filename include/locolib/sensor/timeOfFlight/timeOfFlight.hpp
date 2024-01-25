#pragma once

#include "locolib/units/units.hpp"
#include "locolib/time.hpp"
#include "locolib/sensor/sensorModel.hpp"
#include "locolib/utils/fieldModel.hpp"

namespace Loco {

	class TimeOfFlight : public SensorModel {
	private:
		FieldModel& model;
		QLength maxLength;
	public:
		TimeOfFlight() = delete;
		TimeOfFlight(FieldModel& model, QLength maxLength) : model(model) {
			this->maxLength = maxLength;
		}

		virtual QLength getReading() {
			return 0.0;
		}

		virtual QLength getStd() {
			return 0.0;
		}

		QLength getPredictedReading(const Particle &particle) {

		}

		double confidence(const Loco::Particle &particle) final {
			QLength predictedReading = this->getPredictedReading(particle);
			QLength actualReading = this->getReading();
		}
	};
}
