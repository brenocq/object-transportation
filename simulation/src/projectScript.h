//--------------------------------------------------
// Box Pushing
// projectScript.h
// Date: 2022-10-31
//--------------------------------------------------
#ifndef PROJECT_SCRIPT_H
#define PROJECT_SCRIPT_H
#include <atta/script/projectScript.h>
#include "nlohmann/json.hpp"

namespace scr = atta::script;

class ProjectScript : public scr::ProjectScript {
  public:
    //---------- Simulation ----------//
    void onLoad() override;
    void onUnload() override;
    void onStart() override;
    void onStop() override;
    void onAttaLoop() override;

    //---------- UI ----------//
    void onUIRender() override;

  private:
    // Map handling
    void selectMap(std::string mapName);
    void resetMap();
    // Pusher handling
    void randomizePushers();

    //---------- Experiments ----------//
    void runExperiments();

    //---------- UI ----------//
    void uiControl();
    void uiExperiment();
    void uiPusherInspector();
    void drawerPusherLines();
    void drawerPathLines();

    bool _runExperiments;
    int _currentExperiment;
    int _currentRepetition;
    std::string _currentMap;
    std::vector<atta::vec2> _objectPath;
    nlohmann::json _experimentResults;
};

ATTA_REGISTER_PROJECT_SCRIPT(ProjectScript)

#endif // PROJECT_SCRIPT_H
