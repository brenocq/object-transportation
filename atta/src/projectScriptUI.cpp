//--------------------------------------------------
// Box Pushing
// projectScriptUI.cpp
// Date: 2022-11-27
//--------------------------------------------------

void ProjectScript::uiControl() {
    ImGui::Text("Control");

    //----- Select map -----//
    static const char* options[] = {"reference", "middle", "corner", "2-corners"};
    int selectedMap = 0;
    for (int i = 0; i < 4; i++)
        if (_currentMap == std::string(options[i])) {
            selectedMap = i;
            break;
        }

    ImGui::SetNextItemWidth(100.0f);
    if (ImGui::Combo("Map##ComboMap", &selectedMap, options, 4)) {
        selectMap(options[selectedMap]);
    }

    //----- Randomize pusher -----//
    if (ImGui::Button("Randomize pushers"))
        randomizePushers();
}

void ProjectScript::uiExperiment() {
    ImGui::Text("Experiments");

    if (!_runExperiments) {
        if (ImGui::Button("Start experiments")) {
            if (atta::Config::getState() != atta::Config::State::IDLE) {
                evt::SimulationStop e;
                evt::publish(e);
            }
            _runExperiments = true;
            _currentExperiment = 0;
            _currentRepetition = 0;

            _experimentResults["config"] = {};
            _experimentResults["repetitions"] = {};
        }
    } else {
        if (ImGui::Button("Stop experiments")) {
            if (atta::Config::getState() != atta::Config::State::IDLE) {
                evt::SimulationStop e;
                evt::publish(e);
            }
            _runExperiments = false;
            _currentExperiment = 0;
            _currentRepetition = 0;
        } else {
            ImGui::Text("Experiment %d/%d", _currentExperiment + 1, experiments.size());
            ImGui::Text("Repetition %d/%d", _currentRepetition + 1, experiments[_currentExperiment].numRepetitions);
            ImGui::Text("Parameters:", _currentRepetition + 1, experiments[_currentExperiment].numRepetitions);
            ImGui::Text(" - Num robots: %d", experiments[_currentExperiment].numRobots);
            ImGui::Text(" - Map: %s", experiments[_currentExperiment].map.c_str());
            ImGui::Text(" - Script: %s", experiments[_currentExperiment].script.c_str());
        }
    }
}

void ProjectScript::uiPusherInspector() {
    ImGui::Text("Inspector");
    if (atta::Config::getState() != atta::Config::State::IDLE) {
        cmp::Entity selected = cmp::getSelectedEntity();
        cmp::Factory* factory = cmp::getFactory(pusherProto);
        if (factory->isRootClone(selected)) {
            cmp::Entity clone = selected;
            ImGui::Text("Pusher %d", clone.getId());

            // Get camera components
            cmp::Entity cameras = clone.getChild(0);
            std::array<cmp::CameraSensor*, 4> cams;
            for (unsigned i = 0; i < cams.size(); i++)
                cams[i] = cameras.getChild(i).get<cmp::CameraSensor>();

            // Get sensor module camera info
            std::vector<sns::CameraInfo>& snsCams = sns::getCameraInfos();

            // Show camera images
            for (auto cam : cams)
                if (cam->captureTime >= 0.0f)
                    for (uint32_t i = 0; i < snsCams.size(); i++)
                        if (snsCams[i].component == cam) {
                            ImGui::Image(snsCams[i].renderer->getImGuiTexture(), ImVec2(75, 75));
                            if (cam != cams.back())
                                ImGui::SameLine(0.0f, 0.0f);
                        }
        }
    }
}

void ProjectScript::drawerPathLines() {
    gfx::Drawer::clear("path");
    for (int i = 0; i < int(_objectPath.size()) - 1; i++) {
        gfx::Drawer::Line line;
        line.p0 = atta::vec3(_objectPath[i], 0.1f);
        line.p1 = atta::vec3(_objectPath[i + 1], 0.1f);
        line.c0 = line.c1 = {objectColor.r / 255.0f, objectColor.g / 255.0f, objectColor.b / 255.0f, 1};
        gfx::Drawer::add(line, "path");
    }
}

void ProjectScript::drawerPusherLines() {
    // Draw direction lines
    gfx::Drawer::clear("directions");
    if (atta::Config::getState() != atta::Config::State::IDLE) {
        for (cmp::Entity clone : cmp::getFactory(pusherProto)->getClones()) {
            auto t = clone.get<cmp::Transform>();
            auto p = clone.get<PusherComponent>();
            float ang = 0.0f;
            const float length = 0.1f;

            gfx::Drawer::Line line;
            line.p0 = t->position;

            // Draw goal line
            if (!std::isnan(p->goalDirection)) {
                ang = t->orientation.get2DAngle() - p->goalDirection;
                line.p1 = line.p0 + length * atta::vec3(std::cos(ang), std::sin(ang), 0);
                line.c0 = line.c1 = {goalColor.r / 255.0f, goalColor.g / 255.0f, goalColor.b / 255.0f, 1};
                gfx::Drawer::add(line, "directions");
            }

            // Draw object line
            if (!std::isnan(p->objectDirection)) {
                ang = t->orientation.get2DAngle() - p->objectDirection;
                line.p1 = line.p0 + length * atta::vec3(std::cos(ang), std::sin(ang), 0);
                line.c0 = line.c1 = {objectColor.r / 255.0f, objectColor.g / 255.0f, objectColor.b / 255.0f, 1};
                gfx::Drawer::add(line, "directions");
            }

            // Draw push line
            if (!std::isnan(p->pushDirection)) {
                ang = t->orientation.get2DAngle() - p->pushDirection;
                line.p1 = line.p0 + length * atta::vec3(std::cos(ang), std::sin(ang), 0);
                line.c0 = line.c1 = {pusherColor.r / 255.0f, pusherColor.g / 255.0f, pusherColor.b / 255.0f, 1};
                gfx::Drawer::add(line, "directions");
            }
        }
    }
}
