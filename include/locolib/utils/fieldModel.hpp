#pragma once

#include "locolib/localization/particleFilter/particle.hpp"
#include <limits>

namespace Loco {

	class FieldModel {
		std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> lines;
	public:
		FieldModel(std::initializer_list<std::pair<Eigen::Vector2d, Eigen::Vector2d>> lines) {
			this->lines = lines;
		}

		// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
		QLength getDistanceToObstacle(const Particle& particle) {
			Eigen::Vector2d point1 = particle.getState().block<2, 1>(0, 0);
			Eigen::Vector2d point2 = Eigen::Vector2d(cos(particle.getState().z()), sin(particle.getState().z())) + point1;

			QLength minLength = std::numeric_limits<double>::quiet_NaN();

			for (const auto& line : lines) {

				Eigen::Vector2d point3 = line.first;
				Eigen::Vector2d point4 = line.second;

				// manually putting in to reduce overhead from creating Eigen matrix
				double t = (((point1.x() - point3.x()) * (point3.y() - point4.y())) - ((point1.y() - point3.y()) * (point3.x() - point4.x())))
						/ (((point1.x() - point2.x()) * (point3.y() - point4.y())) - ((point1.y() - point2.y()) * (point3.x() - point4.x())));

				double u = (((point1.x() - point2.y()) * (point1.y() - point3.y())) - ((point1.y() - point2.y()) * (point1.x() - point3.x())))
						   / (((point1.x() - point2.x()) * (point3.y() - point4.y())) - ((point1.y() - point2.y()) * (point3.x() - point4.x())));

				if (t >= 0 && 0 <= u  && u <= 1) {
					if (isnan(minLength.getValue()) || minLength.getValue() > t) {
						minLength = t;
					}
				}
			}

			return minLength;
		}

		static Eigen::Vector2d minPosition;
		static Eigen::Vector2d maxPosition;
	};

	Eigen::Vector2d FieldModel::minPosition = Eigen::Vector2d(-1.8, -1.8);
	Eigen::Vector2d FieldModel::maxPosition = Eigen::Vector2d(1.8, 1.8);
}