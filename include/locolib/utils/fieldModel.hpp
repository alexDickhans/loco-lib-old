#pragma once

#include "locolib/localization/particleFilter/particle.hpp"

namespace Loco {


	class FieldModel {

	public:
		QLength getDistance(const Particle& particle) {
			return std::nullopt;
		}

		static Eigen::Vector2d minPosition;
		static Eigen::Vector2d maxPosition;
	};

	Eigen::Vector2d FieldModel::minPosition = Eigen::Vector2d(-1.8, -1.8);
	Eigen::Vector2d FieldModel::maxPosition = Eigen::Vector2d(1.8, 1.8);
}