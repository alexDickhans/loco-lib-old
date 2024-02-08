#pragma once

#include "locolib/units/units.hpp"

namespace Loco {
	class DeadWheel {
	private:
		QLength lastReading;
		QLength delta;
	protected:
		void updateReadingDistance(QLength const newReading) {
			delta = newReading - lastReading;
			lastReading = newReading;
		}

		void updateReading(Angle angle) {
			updateReadingDistance(angle.getValue() * radius.getValue());
		}

		QLength radius;
	public:
		DeadWheel() = delete;
		explicit DeadWheel(QLength radius) :
			radius(radius),
			lastReading(0.0),
			delta(0.0) {

		}

		virtual void update() {}

		QLength getDisplacement() const {
			return lastReading;
		}

		QLength getDelta() const {
			return delta;
		}

		virtual QLength getTickLength() const {
			return 0.0;
		}
	};
}
