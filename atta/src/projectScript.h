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
    void onStart() override;
    void onStop() override;
    void onUpdateBefore(float dt) override;

    //---------- UI ----------//
    void onUIRender() override;

  private:
    void createMap();
    void destroyMap();

    std::string _currentMap;
};

ATTA_REGISTER_PROJECT_SCRIPT(ProjectScript)

#endif // PROJECT_SCRIPT_H
