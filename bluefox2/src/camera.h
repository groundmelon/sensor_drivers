#pragma once

#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <cassert>
using namespace std;

#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

class BluefoxCamera
{
  public:
    BluefoxCamera(mvIMPACT::acquire::Device *_device);

    bool ready();

    void loop();

    vector<unsigned char> getImg();

  private:
    mvIMPACT::acquire::Device *device;
    mvIMPACT::acquire::FunctionInterface fi;
    mvIMPACT::acquire::SettingsBlueFOX s;
    const Request *pRequest = 0;
    const unsigned int timeout_ms = 0;

    std::thread t;

    std::mutex barrier;
    queue<vector<unsigned char>> data_q;
};

class BluefoxManager
{
  public:
    BluefoxManager();
    int getImgCnt();
    bool ready();
    vector<unsigned char> getImg();

  private:
    mvIMPACT::acquire::DeviceManager devMgr;
    vector<BluefoxCamera *> bluefoxCameras;
};
