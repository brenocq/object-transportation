//--------------------------------------------------
// Box Pushing
// projectScriptUI.cpp
// Date: 2022-11-27
// By Breno Cunha Queiroz
//--------------------------------------------------

void ProjectScript::uiControl() {
    ImGui::Text("Control");

    //----- Select map -----//
    static std::vector<std::string> options = {"Reference", "Middle", "Corner", "2 Corners"};
    ImGui::SetNextItemWidth(100.0f);
    if (ImGui::BeginCombo("Map##ComboMap", _currentMap.c_str())) {
        for (int i = 0; i < options.size(); i++) {
            const bool selected = (options[i] == _currentMap);
            if (ImGui::Selectable(options[i].c_str(), selected)) {
                _currentMap = options[i];
                selectMap(_currentMap);
            }
            if (selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    //----- Randomize pusher -----//
    if (ImGui::Button("Randomize pushers"))
        randomizePushers();
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
            std::array<cmp::Camera*, 4> cams;
            for (unsigned i = 0; i < cams.size(); i++)
                cams[i] = cameras.getChild(i).get<cmp::Camera>();

            // Get sensor module camera info
            std::vector<sns::CameraInfo>& snsCams = sns::getCameraInfos();

            // Show camera images
            for (auto cam : cams)
                for (uint32_t i = 0; i < snsCams.size(); i++)
                    if (snsCams[i].component == cam) {
                        ImGui::Image(snsCams[i].renderer->getImGuiTexture(), ImVec2(75, 75));
                        if (cam != cams.back())
                            ImGui::SameLine(0.0f, 0.0f);
                    }
        }
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
        }
    }
}
