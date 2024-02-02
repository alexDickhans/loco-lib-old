#pragma once

#include "pros/gps.hpp"
#include "../sensorModel.hpp"

namespace Loco {
	class Gps : public SensorModel {
	private:
		pros::Gps& gps;
	public:
		Gps(pros::Gps& gps) : gps(gps) {

		}

		double confidence(const Loco::Particle &particle) override {

		}

		double getWeight() override {

		}
	};
}