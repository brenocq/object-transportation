//--------------------------------------------------
// Box Pushing
// projectScriptExperiments.cpp
// Date: 2023-01-29
//--------------------------------------------------

void ProjectScript::runExperiments() {
    if (_runExperiments) {
        const Experiment exp = experiments[_currentExperiment];
        const atta::vec2 objPos = atta::vec2(object.get<cmp::Transform>()->position);
        const atta::vec2 goalPos = atta::vec2(goal.get<cmp::Transform>()->position);
        const atta::vec2 objScale = atta::vec2(object.get<cmp::Transform>()->scale.x);
        const atta::vec2 goalScale = atta::vec2(goal.get<cmp::Transform>()->scale.x);
        const float pusherDiam = pusherProto.get<cmp::Transform>()->scale.x;
        const float gap = 0.05;
        //const float minDist = (goalScale.x + objScale.length()) * 0.5;// <-- Box
        const float minDist = (goalScale.x + objScale.x) * 0.5 + pusherDiam + gap;// <-- Circle

        // If last experiment finished (simulation not running), start new one
        if (atta::Config::getState() == atta::Config::State::IDLE) {
            // Set parameters
            pusherProto.get<cmp::Prototype>()->maxClones = exp.numRobots;
            pusherProto.get<cmp::Script>()->set(exp.script);
            selectMap(exp.map);

            // JSON config
            if (_currentRepetition == 0) {
                nlohmann::json experimentConfig = {};
                experimentConfig["numRepetitions"] = exp.numRepetitions;
                experimentConfig["numRobots"] = exp.numRobots;
                experimentConfig["controller"] = exp.script;
                experimentConfig["map"] = {};
                experimentConfig["map"]["name"] = exp.map;
                experimentConfig["map"]["goal"] = {maps[exp.map].goalPos.x, maps[exp.map].goalPos.y};
                experimentConfig["map"]["object"] = {maps[exp.map].objectPos.x, maps[exp.map].objectPos.y};
                experimentConfig["timeout"] = exp.timeout;
                experimentConfig["timeStep"] = atta::Config::getDt();
                experimentConfig["minObjectGoalDist"] = minDist;
                _experimentResults["config"] = experimentConfig;
            }

            // JSON repetition
            _experimentResults["repetitions"] += {};

            // Start simulation
            evt::SimulationStart e;
            evt::publish(e);
        }

        // Check stop condition
        float dist = (objPos - goalPos).length();
        bool success = dist <= minDist;
        if (atta::Config::getTime() > exp.timeout || success) {
            // JSON log result
            _experimentResults["repetitions"].back()["success"] = success;
            _experimentResults["repetitions"].back()["time"] = atta::Config::getTime();
            _experimentResults["repetitions"].back()["distance"] = dist;
            _experimentResults["repetitions"].back()["path"] = {};
            for (atta::vec2 pos : _objectPath) {
                nlohmann::json jsonPos = {};
                jsonPos += pos.x;
                jsonPos += pos.y;
                _experimentResults["repetitions"].back()["path"] += jsonPos;
            }

            // Stop simulation
            evt::SimulationStop e;
            evt::publish(e);

            // Advance repetition
            _currentRepetition++;
            if (_currentRepetition == exp.numRepetitions) {
                fs::create_directory("experiments");
                fs::path file = fs::path("experiments") / std::string(exp.map + "-" + exp.script + "-" + std::to_string(exp.numRobots) + "_robots" +
                                                                      "-" + std::to_string(exp.numRepetitions) + "_rep.json");
                std::ofstream out(file);
                LOG_INFO("ProjectScript", "Experiment [w]$0[] saved to [w]$1[]", file.stem().string(), fs::absolute(file));
                out << _experimentResults;
                _experimentResults = {};

                // Advance experiment
                _currentExperiment++;
                _currentRepetition = 0;
                if (_currentExperiment == experiments.size()) {
                    _currentExperiment = 0;
                    _runExperiments = false;
                    // Finished running experiments
                }
            }
        }
    }
}
