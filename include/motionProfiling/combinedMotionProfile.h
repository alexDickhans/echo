#pragma once

#include "motionProfile.h"
#include "pathCommands.h"

class CombinedMotionProfile : public MotionProfile {
private:
	std::vector<MotionProfile*> motionProfiles;
	std::vector<std::pair<double, std::string>> commands;
	size_t commandIndex;
public:
	explicit CombinedMotionProfile(std::vector<MotionProfile *> motion_profiles)
		: motionProfiles(std::move(motion_profiles)) {
	}

	void resetCommands() {
		commandIndex = 0;
	}

	std::optional<MotionCommand> get(const QTime t) override {
		QTime accumulatedTime = 0.0;

		double accumulatedT = 0.0;

		for (auto mp : this->motionProfiles) {
			if (accumulatedTime + mp->getDuration() > t) {
				if (const auto currentProfile = mp->get(t - accumulatedTime)) {
					auto result = currentProfile.value();

					result.desiredT += accumulatedT;

					while (commandIndex < commands.size() && commands[commandIndex].first < result.desiredT) {
						PathCommands::schedule(commands[commandIndex].second);

						commandIndex ++;
					}

					return result;
				}
				return std::nullopt;
			}

			accumulatedTime += mp->getDuration();
			accumulatedT += mp->maxT();
		}

		return std::nullopt;
	}

	[[nodiscard]] QTime getDuration() const override {
		QTime totalTime = 0.0;

		for (const auto motionProfile : this->motionProfiles) {
			totalTime += motionProfile->getDuration();
		}

		return totalTime;
	}

	void addMP(MotionProfile* profile) {
		this->motionProfiles.emplace_back(profile);
	}

	double maxT() override {
		double accumulatedT = 0.0;

		for (auto motion_profile : this->motionProfiles) {
			accumulatedT += motion_profile->maxT();
		}

		return accumulatedT;
	}

	void addCommand(double t, const std::string& name) {
		this->commands.emplace_back(t, name);
	}

	size_t size() {
		return this->motionProfiles.size();
	}

	~CombinedMotionProfile() override = default;
};