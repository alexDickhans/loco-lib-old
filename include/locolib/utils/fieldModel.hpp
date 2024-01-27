#pragma once

#include "locolib/localization/particleFilter/particle.hpp"
#include <limits>

namespace Loco {


	class FieldModel {

	public:
		QLength getDistance(const Particle& particle) {
			return std::numeric_limits<double>::quiet_NaN();
		}

		static Eigen::Vector2d minPosition;
		static Eigen::Vector2d maxPosition;
	};

	Eigen::Vector2d FieldModel::minPosition = Eigen::Vector2d(-1.8, -1.8);
	Eigen::Vector2d FieldModel::maxPosition = Eigen::Vector2d(1.8, 1.8);
}