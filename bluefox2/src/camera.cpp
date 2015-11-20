#include "camera.h"

BluefoxCamera::BluefoxCamera(mvIMPACT::acquire::Device *_device)
    : device(_device), fi(_device), s(_device)
{
    cout << device->family.read() << "(" << device->serial.read() << ")" << endl;

    // basic setting
    s.cameraSetting.restoreDefault();
    s.cameraSetting.expose_us.write(10000);
    s.cameraSetting.triggerSource.write(mvIMPACT::acquire::ctsDigIn0);
    s.cameraSetting.triggerMode.write(mvIMPACT::acquire::ctmOnHighLevel);

    int result = DMR_NO_ERROR;
    SystemSettings ss(device);
    const int REQUEST_COUNT = ss.requestCount.read();
    for (int i = 0; i < REQUEST_COUNT; i++)
    {
        result = fi.imageRequestSingle();
        if (result != DMR_NO_ERROR)
        {
            cout << "Error while filling the request queue: " << ImpactAcquireException::getErrorCodeAsString(result) << endl;
        }
    }

    t = std::thread(&BluefoxCamera::loop, this);
}

void BluefoxCamera::loop()
{
    while (true)
    {
        int requestNr = INVALID_ID;
        requestNr = fi.imageRequestWaitFor(-1);
        if (fi.isRequestNrValid(requestNr))
        {
            pRequest = fi.getRequest(requestNr);
            if (pRequest->isOK())
            {
                vector<unsigned char> data(pRequest->imageHeight.read() * pRequest->imageWidth.read());
                memcpy(data.data(), pRequest->imageData.read(), data.size());
                std::lock_guard<std::mutex> lock(barrier);
                data_q.push(std::move(data));
                assert(data.size() == 0);
                cout << "device: " << device->serial.read() << ", size: " << data_q.size() << endl;
            }
            else
            {
                cout << "device: " << device->serial.read() << ", Error: " << pRequest->requestResult.readS() << endl;
            }
            if (fi.isRequestNrValid(requestNr))
                fi.imageRequestUnlock(requestNr);
            fi.imageRequestSingle();
        }
        else
        {
            // If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
            // additional information under TDMR_ERROR in the interface reference (
            puts("failed");
            cout << "imageRequestWaitFor failed (" << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString(requestNr) << ", device " << device->serial.read() << ")"
                 << ", timeout value too small?" << endl;
        }
    }
}

bool BluefoxCamera::ready()
{
    std::lock_guard<std::mutex> lock(barrier);
    return data_q.size() != 0;
}

vector<unsigned char> BluefoxCamera::getImg()
{
    std::lock_guard<std::mutex> lock(barrier);
    vector<unsigned char> img(std::move(data_q.front()));
    assert(data_q.front().size() == 0);
    data_q.pop();
    return img;
}

BluefoxManager::BluefoxManager()
{
    for (unsigned int i = 0; i < devMgr.deviceCount(); i++)
    {
        try
        {
            devMgr[i]->open();
        }
        catch (const ImpactAcquireException &e)
        {
            cout << "An error occurred while opening the device " << devMgr[i]->serial.read()
                 << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << ")). Terminating thread." << endl;
        }

        bluefoxCameras.push_back(new BluefoxCamera(devMgr[i]));
    }
}

int BluefoxManager::getImgCnt()
{
    return devMgr.deviceCount();
}

bool BluefoxManager::ready()
{
    for (auto &it : bluefoxCameras)
        if (!(it->ready()))
            return false;
    return true;
}

vector<unsigned char> BluefoxManager::getImg()
{
    vector<unsigned char> img;
    for (auto &it : bluefoxCameras)
    {
        vector<unsigned char> sub_img = it->getImg();
        img.insert(img.end(), sub_img.begin(), sub_img.end());
    }
    return img;
}
