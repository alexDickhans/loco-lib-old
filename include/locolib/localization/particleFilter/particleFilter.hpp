#pragma once

#include <vector>
#include "particle.hpp"
#include <random>
#include "locolib/prediction/posePredictor.hpp"
#include "locolib/units/units.hpp"

namespace Loco {
	template<uint32_t size>
	class ParticleFilter {
	private:
		std::array<Particle, size> particles;
		std::default_random_engine generator;
		std::uniform_int_distribution<int> randomItemDistribution{0, size};
		std::normal_distribution<double> startingNormal{0.0, 1.0};
		std::uniform_real_distribution<double> startingUniform;
		OrientationSource& orientationSource;
		PosePredictor& posePredictor;
		Particle bestParticle;
		std::vector<SensorModel*> sensors;
		double totalWeight = 0.0;
	public:
		ParticleFilter(OrientationSource& orientationSource, PosePredictor& posePredictor) : orientationSource(orientationSource), posePredictor(posePredictor) {
			this->initializeUniform(FieldModel::minPosition, FieldModel::maxPosition);
		}

		void initializeUniform(const Eigen::Vector2d& smallBound, const Eigen::Vector2d& largeBound) {
			Eigen::Vector2d boundDifference = largeBound - smallBound;
			Eigen::Matrix2d multiplierMatrix = Eigen::Matrix2d({{boundDifference.x(), 0},
																{0, boundDifference.y()}});
			Eigen::Vector2d middlePoint = smallBound + 0.5 * boundDifference;

			for (size_t i = 0; i < size; i++) {
				auto randVector = middlePoint + multiplierMatrix * Eigen::Vector2d::Random();
				particles.at(i) = Eigen::Vector3d(randVector.x(), randVector.y(), orientationSource.getAngle().getValue());
			}
		}

		void initializeNormal(Eigen::Vector2d startingPosition, Eigen::Matrix2d covariance, Eigen::Vector2d smallestBound, Eigen::Vector2d largestBound) {
			for (size_t i = 0; i < size; i++) {
				particles.at(i) = Particle(startingPosition + (covariance * Eigen::Vector2d(startingNormal(generator), startingNormal(generator))));
			}
		}

		void setOrientation(Angle orientation) {
			for (auto &item: particles) {
				item.setAngle(orientation);
			}
		}

		void setOrientation() {
			this->setOrientation(orientationSource.getAngle());
		}

		void moveParticles(Eigen::Vector3d translation, Eigen::Matrix3d covariance) {
			for (auto &item: particles) {
				item.setState(item.getState() + translation + Eigen::Vector3d::Random() * covariance);
			}
		}

		void weightParticles() {
			double highestWeight = 0.0;
			totalWeight = 0.0;

			for (auto &particle: particles) {
				double weight = 0.0;
				double weightModifiers = 0.0;

				for (auto &sensor: sensors) {
					weight += sensor->confidence(particle) * sensor->getWeight();
					weightModifiers += sensor->getWeight();
				}

				particle.setWeight(weight/weightModifiers);
				totalWeight += particle.getWeight();

				if (particle.getWeight() > highestWeight) {
					highestWeight = particle.getWeight();
					bestParticle = particle;
				}
			}
		}

		void resampleParticles() {



			for (size_t i = 0; i < size; i++) {

			}
		}

		void update() {
			QTime timeDelta = 20_ms;

			this->setOrientation();

			auto odomPoseDelta = posePredictor.getDeltaAtTimeDelta(timeDelta);

			this->moveParticles(odomPoseDelta, Eigen::Matrix<double, 3, 3>({{odomPoseDelta.x(), 0.0, 0.0}, {0.0, odomPoseDelta.y(), 0.0}, {0.0, 0.0, 0.0}}));

			this->weightParticles();


		}

		Particle getBestParticle() {
			return bestParticle;
		}

		Particle getRandomParticle() {
			return particles.at(randomItemDistribution(generator));
		}
	};
}