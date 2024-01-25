#pragma once

#include "orientationSource/orientationSource.hpp"

namespace Loco {
	class PosePredictor {
	private:
		Eigen::Vector3d currentPose;
		std::vector<Eigen::Vector3d> lastPositions;

		OrientationSource& orientationSource;

		const int historySize = 15;
	protected:

		void updatePose(Eigen::Vector3d poseDelta) {
			lastPositions.emplace_back(currentPose);
			currentPose = poseDelta + currentPose;

			if (lastPositions.size() > historySize) {
				lastPositions.erase(lastPositions.begin());
			}
		}

		Angle getAngle() {
			return orientationSource.getAngle();
		}
	public:
		PosePredictor() = delete;
		PosePredictor(Eigen::Vector3d startPose, OrientationSource& orientationSource) : currentPose(startPose), orientationSource(orientationSource) {
		}

		virtual void update() { updatePose(Eigen::Vector3d()); }

		Eigen::Vector3d getCurrentPose() {
			return currentPose;
		}

		Eigen::Vector3d getDeltaAtTimeDelta(QTime deltaTime) {
			if (lastPositions.size() == 0) {
				return Eigen::Vector3d();
			}

			QTime currentTime = Time::currentTime();

			for (int i = lastPositions.size()-1; i >= 0; i--) {
				if ((lastPositions.size()-i) * 10_ms > currentTime) {
					return this->getCurrentPose() - lastPositions.at(i);
				}
			}

			return this->getCurrentPose() - lastPositions.at(0);
		}



		void reset() {
			lastPositions.clear();
		}
	};
}