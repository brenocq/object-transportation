//--------------------------------------------------
// Box Pushing
// projectScript.h
// Date: 2022-10-31
// By Breno Cunha Queiroz
//--------------------------------------------------
#ifndef PROJECT_SCRIPT_H
#define PROJECT_SCRIPT_H
#include <atta/script/projectScript.h>

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

    //---------- UI ----------//
    void uiControl();
    void uiPusherInspector();
    void drawerPusherLines();
    void drawerPathLines();

    std::string _currentMap;
    std::vector<atta::vec2> _objectPath;
};

ATTA_REGISTER_PROJECT_SCRIPT(ProjectScript)

#endif // PROJECT_SCRIPT_H
