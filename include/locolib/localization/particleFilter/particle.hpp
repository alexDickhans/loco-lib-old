#pragma once

#include "Eigen/Dense"

namespace Loco {
	class Particle {
	private:
		Eigen::Vector3d state;
		double weight{1.0};
	public:
		Particle() = default;
		Particle(Eigen::Vector3d state, double weight = 1.0) {
			this->state = state;
			this->weight = weight;
		}

		[[nodiscard]] Eigen::Vector3d getState() const {
			return state;
		}

		[[nodiscard]] double getWeight() const {
			return weight;
		}

		void setState(Eigen::Vector3d const state) {
			this->state = state;
		}

		void setAngle(Angle angle) {
			this->state.z() = angle.getValue();
		}

		void setWeight(double const weight) {
			this->weight = weight;
		}
	};
}