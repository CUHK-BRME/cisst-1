/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$
  
  Author(s):  Balazs Vagvolgyi
  Created on: 2006 

  (C) Copyright 2006-2008 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include <cisstCommon.h>
#include <cisstStereoVision/svlFilterSourceVideoCapture.h>
#include <cisstOSAbstraction/osaThread.h>
#include <cisstOSAbstraction/osaSleep.h>
#include "svlVidCapSrcInitializer.h"

#ifdef _MSC_VER
    // Quick fix for Visual Studio Intellisense:
    // The Intellisense parser can't handle the CMN_UNUSED macro
    // correctly if defined in cmnPortability.h, thus
    // we should redefine it here for it.
    // Removing this part of the code will not effect compilation
    // in any way, on any platforms.
    #undef CMN_UNUSED
    #define CMN_UNUSED(argument) argument
#endif

#define MAX_PROPERTIES_BUFFER_SIZE      65536


/*******************************************/
/*** svlFilterSourceVideoCapture class *****/
/*******************************************/

CMN_IMPLEMENT_SERVICES(svlFilterSourceVideoCapture)

svlFilterSourceVideoCapture::svlFilterSourceVideoCapture() :
    svlFilterSourceBase(),
    OutputImage(0),
    EnumeratedDevices(0),
    NumberOfEnumeratedDevices(-1),
    FormatList(0),
    FormatListSize(0)
{
    AddOutput("output", true);
    SetAutomaticOutputType(false);

    InitializeCaptureAPIs();
}

svlFilterSourceVideoCapture::svlFilterSourceVideoCapture(unsigned int channelcount) :
    svlFilterSourceBase(),
    OutputImage(0),
    EnumeratedDevices(0),
    NumberOfEnumeratedDevices(-1),
    FormatList(0),
    FormatListSize(0)
{
    AddOutput("output", true);
    SetAutomaticOutputType(false);

    InitializeCaptureAPIs();

    SetChannelCount(channelcount);
}

svlFilterSourceVideoCapture::~svlFilterSourceVideoCapture()
{
    Release();

    if (OutputImage) {
        delete OutputImage;

        unsigned int i;

        if (DeviceID) delete [] DeviceID;
        if (InputID) delete [] InputID;
        if (Trigger) delete [] Trigger;
        if (Format) {
            for (i = 0; i < NumberOfChannels; i ++) {
                if (Format[i]) delete Format[i];
            }
            delete [] Format;
        }
        if (Properties) {
            for (i = 0; i < NumberOfChannels; i ++) {
                if (Properties[i]) delete Properties[i];
            }
            delete [] Properties;
        }
        if (DevSpecConfigBuffer) {
            for (i = 0; i < NumberOfChannels; i ++) {
                if (DevSpecConfigBuffer[i]) delete DevSpecConfigBuffer[i];
            }
            delete [] DevSpecConfigBuffer;
        }
        if (DevSpecConfigBufferSize) delete [] DevSpecConfigBufferSize;
        if (APIChannelID) delete [] APIChannelID;
        if (APIDeviceID) delete [] APIDeviceID;
        if (API) delete [] API;
        if (EnumeratedDevices) delete [] EnumeratedDevices;
        if (FormatList) {
            for (int j = 0; j < NumberOfEnumeratedDevices; j ++) {
                if (FormatList[j]) delete [] FormatList[j];
            }
            delete [] FormatList;
        }
        if (FormatListSize) delete [] FormatListSize;
    }
}

int svlFilterSourceVideoCapture::SetChannelCount(unsigned int channelcount)
{
    if (OutputImage) return SVL_FAIL;

    if (channelcount == 1) {
        GetOutput()->SetType(svlTypeImageRGB);
        // forcing output sample to handle external buffers
        OutputImage = new svlSampleImageRGB(false);
    }
    else if (channelcount == 2) {
        GetOutput()->SetType(svlTypeImageRGBStereo);
        // forcing output sample to handle external buffers
        OutputImage = new svlSampleImageRGBStereo(false);
    }
    else return SVL_FAIL;

    NumberOfChannels = channelcount;

    DeviceID = new int[NumberOfChannels];
    InputID = new int[NumberOfChannels];
    Format = new ImageFormat*[NumberOfChannels];
    Properties = new ImageProperties*[NumberOfChannels];
    Trigger = new ExternalTrigger[NumberOfChannels];
    DevSpecConfigBuffer = new unsigned char*[NumberOfChannels];
    DevSpecConfigBufferSize = new unsigned int[NumberOfChannels];
    APIChannelID = new int[NumberOfChannels];
    APIDeviceID = new int[NumberOfChannels];
    API = new int[NumberOfChannels];

    for (unsigned int i = 0; i < NumberOfChannels; i ++) {
        DeviceID[i] = -1;
        InputID[i] = -1;
        Format[i] = 0;
        Properties[i] = 0;
        memset(&(Trigger[i]), 0, sizeof(ExternalTrigger));
        DevSpecConfigBuffer[i] = 0;
        DevSpecConfigBufferSize[i] = 0;
        APIChannelID[i] = -1;
        APIDeviceID[i] = -1;
        API[i] = -1;
    }

    return SVL_OK;
}

int svlFilterSourceVideoCapture::Initialize(svlSample* &syncOutput)
{
    if (OutputImage == 0) return SVL_FAIL;
    syncOutput = OutputImage;

    PlatformType platform;
    unsigned int i;
    int ret = SVL_FAIL;

    Release();

    // Create capture API handlers
    if (CreateCaptureAPIHandlers() != SVL_OK) {
        ret = SVL_FAIL;
        goto labError;
    }

    // Set capture devices
    for (i = 0; i < NumberOfChannels; i ++) {
        if (DeviceObj[API[i]]->SetDevice(APIDeviceID[i], InputID[i], APIChannelID[i]) != SVL_OK) {
            ret = SVL_VCS_UNABLE_TO_SET_INPUT;
            goto labError;
        }
    }

    // Set format if available
    for (i = 0; i < NumberOfChannels; i ++) {
        platform = DeviceObj[API[i]]->GetPlatformType();

        if (platform == WinDirectShow) {
#if (CISST_SVL_HAS_DIRECTSHOW == ON)
            // DirectShow does not use the Format structure.
            // Instead, it has its own custom configuration format.
            if (DevSpecConfigBuffer[i] && DevSpecConfigBufferSize[i] > 0) {
                dynamic_cast<svlVidCapSrcDirectShow*>(DeviceObj[API[i]])->SetMediaType(DevSpecConfigBuffer[i], DevSpecConfigBufferSize[i], APIChannelID[i]);
            }
#endif // CISST_SVL_HAS_DIRECTSHOW
        }
#if (CISST_SVL_HAS_MIL == ON)
        else if (platform == MatroxImaging) {
            // Check if Matrox device supports capture
            if (dynamic_cast<svlVidCapSrcMIL*>(DeviceObj[API[i]])->EnableCapture(APIDeviceID[i]) == false) {
                ret = SVL_VCS_UNABLE_TO_OPEN;
                goto labError;
            }
        }
#endif // CISST_SVL_HAS_MIL
        else {
            if (Format[i]) DeviceObj[API[i]]->SetFormat(Format[i][0], APIChannelID[i]);
        }
    }

    // Set trigger
    for (i = 0; i < NumberOfChannels; i ++) {
        DeviceObj[API[i]]->SetTrigger(Trigger[i], APIChannelID[i]);
    }

    // Open devices
    for (i = 0; i < NumberOfChannels; i ++) {
        if (DeviceObj[API[i]]->Open() != SVL_OK) {
            ret = SVL_VCS_UNABLE_TO_OPEN;
            goto labError;
        }
    }

    // Set image properties if available
    for (i = 0; i < NumberOfChannels; i ++) {
        platform = DeviceObj[API[i]]->GetPlatformType();

        if (platform == WinDirectShow) {
#if (CISST_SVL_HAS_DIRECTSHOW == ON)
            // DirectShow does not use the Properties structure.
            // Instead, it encodes image preperties into the custom
            // configuration format which was already set above.
#endif // CISST_SVL_HAS_DIRECTSHOW
        }
        else {
            if (Properties[i]) DeviceObj[API[i]]->SetImageProperties(Properties[i][0], APIChannelID[i]);
        }
    }

    // Requesting frames from the capture buffer to prepare output sample
    for (i = 0; i < NumberOfChannels; i ++) {
        if (NumberOfChannels == 2) {
            dynamic_cast<svlSampleImageRGBStereo*>(OutputImage)->SetMatrix(DeviceObj[API[i]]->GetLatestFrame(false, APIChannelID[i]), i);
        }
        else {
            dynamic_cast<svlSampleImageRGB*>(OutputImage)->SetMatrix(DeviceObj[API[i]]->GetLatestFrame(false));
        }
    }

    // Start capturing in the background
    for (i = 0; i < NumberOfChannels; i ++) {
        if (DeviceObj[API[i]]->Start() != SVL_OK) {
            ret = SVL_VCS_UNABLE_TO_START_CAPTURE;
            goto labError;
        }
    }

    return SVL_OK;

labError:
    Release();
    return ret;
}

int svlFilterSourceVideoCapture::Process(svlProcInfo* procInfo, svlSample* &syncOutput)
{
    syncOutput = OutputImage;

    svlImageRGB* image;
    unsigned int idx;

    _ParallelLoop(procInfo, idx, NumberOfChannels)
    {
        // Requesting frame from the capture buffer
        image = DeviceObj[API[idx]]->GetLatestFrame(true, APIChannelID[idx]);
        if (image == 0) return SVL_FAIL;
        if (NumberOfChannels == 1) {
            dynamic_cast<svlSampleImageRGB*>(OutputImage)->SetMatrix(image, idx);
        }
        else {
            dynamic_cast<svlSampleImageRGBStereo*>(OutputImage)->SetMatrix(image, idx);
        }
    }

    return SVL_OK;
}

int svlFilterSourceVideoCapture::Release()
{
    for (unsigned int i = 0; i < NumberOfSupportedAPIs; i ++) {

#if (CISST_SVL_HAS_MIL == ON)
        // MIL device object is a singleton, should not be deleted
        if (DeviceObj[i] &&
            DeviceObj[i]->GetPlatformType() == MatroxImaging) {
            dynamic_cast<svlVidCapSrcMIL*>(DeviceObj[i])->Release();
            continue;
        }
#endif // CISST_SVL_HAS_MIL

        // Device object is released through the generic base class
        SupportedAPIs[i]->Delete(DeviceGenObj[i]);
        DeviceGenObj[i] = 0;
        DeviceObj[i] = 0;
    }

    return SVL_OK;
}

void svlFilterSourceVideoCapture::InitializeCaptureAPIs()
{
    NumberOfSupportedAPIs = 0;

    // Enumerate registered APIs
    cmnGenericObject* go;
    svlVidCapSrcBase* api;
    SupportedAPIs.SetSize(256);
    APIPlatforms.SetSize(256);

    // Go through all registered classes
    for (cmnClassRegister::const_iterator iter = cmnClassRegister::begin();
         iter != cmnClassRegister::end();
         iter ++) {

        // Avoid infinite recursion by skipping self
        if ((*iter).first != "svlFilterSourceVideoCapture" &&
            (*iter).second) {

            // Most device objects can be created dynamically
            go = (*iter).second->Create();

            if (go == 0) {
#if (CISST_SVL_HAS_MIL == ON)
                // MIL device object is a singleton, cannot be created dynamically
                if ((*iter).first == "svlVidCapSrcMIL") {
                    go = svlVidCapSrcMIL::GetInstance();
                }
#endif // CISST_SVL_HAS_MIL
            }

            api = dynamic_cast<svlVidCapSrcBase*>(go);

            if (api) {
                SupportedAPIs[NumberOfSupportedAPIs] = (*iter).second;
                APIPlatforms[NumberOfSupportedAPIs] = api->GetPlatformType();
                NumberOfSupportedAPIs ++;
            }

            // Delete method will release only dynamically created objects
            (*iter).second->Delete(go);
        }
    }

    SupportedAPIs.resize(NumberOfSupportedAPIs);
    APIPlatforms.resize(NumberOfSupportedAPIs);

    DeviceGenObj.resize(NumberOfSupportedAPIs);
    DeviceObj.resize(NumberOfSupportedAPIs);
    DeviceGenObj.SetAll(0);
    DeviceObj.SetAll(0);
}

int svlFilterSourceVideoCapture::CreateCaptureAPIHandlers()
{
    int ret = SVL_FAIL;
    unsigned int i, j;
    unsigned int *chperapi = new unsigned int[NumberOfSupportedAPIs];
    for (j = 0; j < NumberOfSupportedAPIs; j ++) chperapi[j] = 0;

    // Make sure devices are enumerated
    GetDeviceList(0);

    // Count the number of video channels requested from capture APIs
    // and create API look up table for easy device handler access
    for (i = 0; i < NumberOfChannels; i ++) {

        if (DeviceID[i] < 0 || DeviceID[i] >= NumberOfEnumeratedDevices) goto labError;

        // Enumerate registered APIs and store all results
        for (j = 0; j < NumberOfSupportedAPIs; j ++) {

            if (EnumeratedDevices[DeviceID[i]].platform == APIPlatforms[j]) {
                // getting API specific device and channel IDs
                APIDeviceID[i] = EnumeratedDevices[DeviceID[i]].id;
                APIChannelID[i] = chperapi[j];

                API[i] = j;
                chperapi[j] ++;
            }
        }
    }

    // Instantiate capture device handlers and
    // set the number of channels requested
    for (j = 0; j < NumberOfSupportedAPIs; j ++) {

        if (chperapi[j] > 0) {

            // Most device objects can be created dynamically
            DeviceGenObj[j] = SupportedAPIs[j]->Create();

            if (DeviceGenObj[j] == 0) {
#if (CISST_SVL_HAS_MIL == ON)
                // MIL device object is a singleton, cannot be created dynamically
                if (APIPlatforms[j] == MatroxImaging) {
                    DeviceGenObj[j] = svlVidCapSrcMIL::GetInstance();
                }
#endif // CISST_SVL_HAS_MIL
            }

            DeviceObj[j] = dynamic_cast<svlVidCapSrcBase*>(DeviceGenObj[j]);

            if (DeviceObj[j] == 0 ||
                DeviceObj[j]->SetStreamCount(chperapi[j]) != SVL_OK) goto labError;
        }
    }

    ret = SVL_OK;

labError:
    if (ret != SVL_OK) {
        for (j = 0; j < NumberOfSupportedAPIs; j ++) {
            // Device object is released through the generic base class
            SupportedAPIs[j]->Delete(DeviceGenObj[j]);
            DeviceGenObj[j] = 0;
            DeviceObj[j] = 0;
        }
    }
    if (chperapi) delete [] chperapi;

    return ret;
}

double svlFilterSourceVideoCapture::GetTargetFrequency()
{
    return -1.0;
}

int svlFilterSourceVideoCapture::SetTargetFrequency(double CMN_UNUSED(hertz))
{
    return SVL_FAIL;
}

int svlFilterSourceVideoCapture::DialogSetup(unsigned int videoch)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (IsInitialized() == true)
        return SVL_ALREADY_INITIALIZED;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;

    int deviceid, inputid;

    std::cout << " === Capture device selection ===" << std::endl;
    deviceid = DialogDevice();
    if (deviceid < 0) return SVL_FAIL;

    std::cout << std::endl << "  ==== Device input selection ====" << std::endl;
    inputid = DialogInput(deviceid);

    SetDevice(deviceid, inputid, videoch);

    std::cout << std::endl << "  ===== Setup capture format =====" << std::endl;
    DialogFormat(videoch);

    if (EnumeratedDevices[DeviceID[videoch]].platform == LinLibDC1394) {
#if (CISST_SVL_HAS_DC1394 == ON)
        std::cout << std::endl << "  ===== Setup external trigger =====" << std::endl;
        DialogTrigger(videoch);
#endif // CISST_SVL_HAS_DC1394
    }
    else {
        // External trigger is not supported in other APIs.
    }

    return SVL_OK;
}

int svlFilterSourceVideoCapture::DialogDevice()
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (IsInitialized() == true)
        return SVL_ALREADY_INITIALIZED;

    int deviceid, listsize;

    listsize = PrintDeviceList();
    if (listsize < 1) {
        std::cout << " -!- No video capture devices have been found." << std::endl;
        return SVL_FAIL;
    }

    std::cout << std::endl << " # Enter device ID: ";
    std::cin >> deviceid;
    if (deviceid < 0) deviceid = 0;
    if (deviceid >= listsize) deviceid = listsize - 1;
    
    return deviceid;
}

int svlFilterSourceVideoCapture::DialogInput(unsigned int deviceid)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (IsInitialized() == true)
        return SVL_ALREADY_INITIALIZED;

    int inputid, listsize;

    listsize = PrintInputList(deviceid);
    if (listsize > 0) {
        std::cout << std::endl << "  # Enter input ID: ";
        std::cin >> inputid;
        if (inputid < 0) inputid = 0;
        if (inputid >= listsize) inputid = listsize - 1;
    }
    else {
        std::cout << "  -!- Input selection not available." << std::endl;
        inputid = 0;
    }

    return inputid;
}

int svlFilterSourceVideoCapture::DialogFormat(unsigned int videoch)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (IsInitialized() == true)
        return SVL_ALREADY_INITIALIZED;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;
    if (DeviceID[videoch] < 0)
        return SVL_VCS_UNABLE_TO_OPEN;

    // Make sure devices are enumerated
    GetDeviceList(0);

    // Check if device ID is in range
    if (DeviceID[videoch] >= NumberOfEnumeratedDevices)
        return SVL_VCS_UNABLE_TO_OPEN;

    // Get capture API platform
    PlatformType platform = EnumeratedDevices[DeviceID[videoch]].platform;

    if (platform == WinDirectShow) {
#if (CISST_SVL_HAS_DIRECTSHOW == ON)
        // Create temporary DirectShow capture module and initialize it
        svlVidCapSrcDirectShow device;
        device.SetStreamCount(1);
        if (device.SetDevice(EnumeratedDevices[DeviceID[videoch]].id,
                             InputID[videoch]) != SVL_OK) return SVL_VCS_UNABLE_TO_OPEN;
        // If device properties are already available, load them up to the device
        if (DevSpecConfigBuffer[videoch] && DevSpecConfigBufferSize[videoch] > 0) {
            device.SetMediaType(DevSpecConfigBuffer[videoch], DevSpecConfigBufferSize[videoch]);
        }
        // Open device
        if (device.Open() != SVL_OK) return SVL_VCS_UNABLE_TO_OPEN;
        // Open Format OLE Dialog and save new parameters
        if (device.ShowFormatDialog(0) == SVL_OK) {
            device.RequestMediaType();
            if (DevSpecConfigBuffer[videoch]) delete [] DevSpecConfigBuffer[videoch];
            DevSpecConfigBuffer[videoch] = 0;
            DevSpecConfigBufferSize[videoch] = 0;
            device.GetMediaType(DevSpecConfigBuffer[videoch], DevSpecConfigBufferSize[videoch]);
        }
        else return SVL_FAIL;
#endif // CISST_SVL_HAS_DIRECTSHOW
    }
    else {
        // No GUI available for other APIs.
        // Ask in command line.

        ImageFormat *formats = 0;
        int i, formatid, formatcount;

        formatcount = PrintFormatList(videoch);
        if (formatcount > 0) {
            std::cout << std::endl << "  # Enter format ID: ";
            std::cin >> formatid;
            if (formatid < 0 || formatid >= formatcount) {
                std::cout << "  -!- Invalid format ID" << std::endl;
                return SVL_FAIL;
            }

            GetFormatList(&formats, videoch);
            if (formats[formatid].custom_mode < 0) {
                SelectFormat(formatid, videoch);
            }
            else {
                int roiwidth, roiheight, roileft, roitop, framerate, colorspace;
                std::cout << "  # Enter ROI width (max=" << formats[formatid].custom_maxwidth << "; unit=" << formats[formatid].custom_unitwidth << "): ";
                std::cin >> roiwidth;
                if (roiwidth < 1 || roiwidth >= static_cast<int>(formats[formatid].custom_maxwidth)) {
                    std::cout << "  -!- Invalid ROI width" << std::endl;
                    return SVL_FAIL;
                }
                std::cout << "  # Enter ROI height (max=" << formats[formatid].custom_maxheight << "; unit=" << formats[formatid].custom_unitheight << "): ";
                std::cin >> roiheight;
                if (roiheight < 1 || roiheight >= static_cast<int>(formats[formatid].custom_maxheight)) {
                    std::cout << "  -!- Invalid ROI height" << std::endl;
                    return SVL_FAIL;
                }
                std::cout << "  # Enter ROI left (max=" << formats[formatid].custom_maxwidth - 1 << "; unit=" << formats[formatid].custom_unitleft << "): ";
                std::cin >> roileft;
                if (roileft < 0 || roileft >= (static_cast<int>(formats[formatid].custom_maxwidth) - 1)) {
                    std::cout << "  -!- Invalid ROI left" << std::endl;
                    return SVL_FAIL;
                }
                std::cout << "  # Enter ROI top (max=" << formats[formatid].custom_maxheight - 1 << "; unit=" << formats[formatid].custom_unittop << "): ";
                std::cin >> roitop;
                if (roitop < 0 || roitop >= (static_cast<int>(formats[formatid].custom_maxheight) - 1)) {
                    std::cout << "  -!- Invalid ROI top" << std::endl;
                    return SVL_FAIL;
                }
                std::cout << "  # Enter framerate [percentage of maximum available framerate] (min=1; max=100): ";
                std::cin >> framerate;
                if (framerate < 1 || framerate > 100) {
                    std::cout << "  -!- Invalid framerate" << std::endl;
                    return SVL_FAIL;
                }
                std::cout << "  == Select color space ==" << std::endl;
                for (i = 0; i < PixelTypeCount && formats[formatid].custom_colorspaces[i] != PixelUnknown; i ++) {
                    std::cout << "  " << i << ") " << GetPixelTypeName(formats[formatid].custom_colorspaces[i]) << std::endl;
                }
                std::cout << "  # Enter color space ID: ";
                std::cin >> colorspace;
                if (colorspace < 0 || colorspace >= i) {
                    std::cout << "  -!- Invalid color space" << std::endl;
                    return SVL_FAIL;
                }

                ImageFormat format;
                memcpy(&format, &(formats[formatid]), sizeof(ImageFormat));
                format.width = roiwidth;
                format.height = roiheight;
                format.colorspace = formats[formatid].custom_colorspaces[colorspace];
                format.custom_roileft= roileft;
                format.custom_roitop = roitop;
                format.custom_framerate = framerate;
                SetFormat(format, videoch);
            }

            ReleaseFormatList(formats);
        }
        else {
            std::cout << "  -!- Format selection not available." << std::endl;
        }
    }

    return SVL_OK;
}

int svlFilterSourceVideoCapture::DialogTrigger(unsigned int videoch)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (IsInitialized() == true)
        return SVL_ALREADY_INITIALIZED;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;
    if (DeviceID[videoch] < 0)
        return SVL_VCS_UNABLE_TO_OPEN;

    // Make sure devices are enumerated
    GetDeviceList(0);

    // Check if device ID is in range
    if (DeviceID[videoch] >= NumberOfEnumeratedDevices)
        return SVL_VCS_UNABLE_TO_OPEN;

    if (EnumeratedDevices[DeviceID[videoch]].platform == LinLibDC1394) {
        ExternalTrigger trigger;
        int ivalue;
        std::string str;

        std::cout << std::endl << "  # Enable external trigger? ['y' or 'n']: ";
        std::cin >> str;
        if (str.compare("y") != 0) {
            memset(&trigger, 0, sizeof(ExternalTrigger));
            std::cout << "    External trigger DISABLED" << std::endl;
            SetTrigger(trigger, videoch);
            return SVL_OK;
        }
        trigger.enable = true;
        std::cout << "    External trigger ENABLED" << std::endl;

        std::cout << "  # Enter trigger mode ['0'-'5' or '14'-'15']: ";
        std::cin >> ivalue;
        if (ivalue < 0) ivalue = 0;
        trigger.mode = ivalue;

        std::cout << "  # Enter trigger source ['0'-'3']: ";
        std::cin >> ivalue;
        if (ivalue < 0) ivalue = 0;
        trigger.source = ivalue;

        std::cout << "  # Enter trigger polarity ['h' or 'l']: ";
        std::cin >> str;
        if (str.compare("h") == 0) {
            trigger.polarity = 1;
            std::cout << "    Trigger polarity set to HIGH" << std::endl;
        }
        else {
            trigger.polarity = 0;
            std::cout << "    Trigger polarity set to LOW" << std::endl;
        }

        SetTrigger(trigger, videoch);
    }
    else {
        std::cout << "  -!- External trigger not supported." << std::endl;
    }

    return SVL_OK;
}

int svlFilterSourceVideoCapture::DialogImageProperties(unsigned int videoch)
{
    // Available only after initialization
    if (IsInitialized() == false)
        return SVL_NOT_INITIALIZED;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;

    // Get capture API platform and API specific device ID
    PlatformType platform = DeviceObj[API[videoch]]->GetPlatformType();

    if (platform == WinDirectShow) {
#if (CISST_SVL_HAS_DIRECTSHOW == ON)
        svlVidCapSrcDirectShow* device = dynamic_cast<svlVidCapSrcDirectShow*>(DeviceObj[API[videoch]]);
        if (device->ShowImageDialog(0, APIChannelID[videoch]) == SVL_OK) {
            // Store, whatever changes have been made
            device->RequestMediaType();
            if (DevSpecConfigBuffer[videoch]) delete [] DevSpecConfigBuffer[videoch];
            DevSpecConfigBuffer[videoch] = 0;
            DevSpecConfigBufferSize[videoch] = 0;
            device->GetMediaType(DevSpecConfigBuffer[videoch], DevSpecConfigBufferSize[videoch]);
        }
#endif // CISST_SVL_HAS_DIRECTSHOW
    }
    else {
        // No GUI available for other APIs
        // Ask in command line.

        printf("  ___________________________________________________________________________\r\n");
        printf("  IMAGE PROPERTIES (channel #%d)                         [press 'q' when done]\r\n", videoch);
        printf("   _________________________________________________________________________\r\n");
        printf("  | Shttr[!] | Gain[@] | WhiteBalance[#] | Brghtns[$] | Gamma[%%] | Satur[^] |\r\n");
        printf("  |    <1 2> |   <3 4> |  <5 6> |  <7 8> |      <9 0> |    <i o> |    <k l> |\r\n");

        int ret = SVL_FAIL;
        svlVidCapSrcDialogThread* proc;
        osaThread* thread;

        proc = new svlVidCapSrcDialogThread(videoch);
        thread = new osaThread;
        thread->Create<svlVidCapSrcDialogThread, svlFilterSourceVideoCapture*>(proc,
                                                                                  &svlVidCapSrcDialogThread::Proc,
                                                                                  this);
        if (proc->WaitForInit()) {
            GetImageProperties(videoch);

            int ch;
            do {
                ch = cmnGetChar();
                switch (ch) {
                    case '!':
                        Properties[videoch]->manual = ~(Properties[videoch]->manual);
                        Properties[videoch]->mask = propShutter;
                    break;

                    case '@':
                        Properties[videoch]->manual = ~(Properties[videoch]->manual);
                        Properties[videoch]->mask = propGain;
                    break;

                    case '#':
                        Properties[videoch]->manual = ~(Properties[videoch]->manual);
                        Properties[videoch]->mask = propWhiteBalance;
                    break;

                    case '$':
                        Properties[videoch]->manual = ~(Properties[videoch]->manual);
                        Properties[videoch]->mask = propBrightness;
                    break;

                    case '%':
                        Properties[videoch]->manual = ~(Properties[videoch]->manual);
                        Properties[videoch]->mask = propGamma;
                    break;

                    case '^':
                        Properties[videoch]->manual = ~(Properties[videoch]->manual);
                        Properties[videoch]->mask = propSaturation;
                    break;

                    case '1':
                        if (Properties[videoch]->shutter >= 10) Properties[videoch]->shutter -= 10;
                        else Properties[videoch]->shutter = 0;
                        Properties[videoch]->mask = propShutter;
                    break;

                    case '2':
                        Properties[videoch]->shutter += 10;
                        Properties[videoch]->mask = propShutter;
                    break;

                    case '3':
                        if (Properties[videoch]->gain >= 5) Properties[videoch]->gain -= 5;
                        else Properties[videoch]->gain = 0;
                        Properties[videoch]->mask = propGain;
                    break;

                    case '4':
                        Properties[videoch]->gain += 5;
                        Properties[videoch]->mask = propGain;
                    break;

                    case '5':
                        if (Properties[videoch]->wb_u_b >= 5) Properties[videoch]->wb_u_b -= 5;
                        else Properties[videoch]->wb_u_b = 0;
                        Properties[videoch]->mask = propWhiteBalance;
                    break;

                    case '6':
                        Properties[videoch]->wb_u_b += 5;
                        Properties[videoch]->mask = propWhiteBalance;
                    break;

                    case '7':
                        if (Properties[videoch]->wb_v_r >= 5) Properties[videoch]->wb_v_r -= 5;
                        else Properties[videoch]->wb_v_r = 0;
                        Properties[videoch]->mask = propWhiteBalance;
                    break;

                    case '8':
                        Properties[videoch]->wb_v_r += 5;
                        Properties[videoch]->mask = propWhiteBalance;
                    break;

                    case '9':
                        if (Properties[videoch]->brightness >= 5) Properties[videoch]->brightness -= 5;
                        else Properties[videoch]->brightness = 0;
                        Properties[videoch]->mask = propBrightness;
                    break;

                    case '0':
                        Properties[videoch]->brightness += 5;
                        Properties[videoch]->mask = propBrightness;
                    break;

                    case 'i':
                        if (Properties[videoch]->gamma >= 1) Properties[videoch]->gamma -= 1;
                        else Properties[videoch]->gamma = 0;
                        Properties[videoch]->mask = propGamma;
                    break;

                    case 'o':
                        Properties[videoch]->gamma += 1;
                        Properties[videoch]->mask = propGamma;
                    break;

                    case 'k':
                        if (Properties[videoch]->saturation >= 5) Properties[videoch]->saturation -= 5;
                        else Properties[videoch]->saturation = 0;
                        Properties[videoch]->mask = propSaturation;
                    break;

                    case 'l':
                        Properties[videoch]->saturation += 5;
                        Properties[videoch]->mask = propSaturation;
                    break;

                    default:
                    break;
                }

                SetImageProperties(videoch);
                osaSleep(0.08);
            } while (ch != 'q');
        }

        proc->Kill();
        thread->Wait();
        delete thread;
        delete proc;

        return ret;
    }

    return SVL_OK;
}

int svlFilterSourceVideoCapture::GetDeviceList(DeviceInfo **deviceinfolist, bool update)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (IsInitialized() == true)
        return SVL_ALREADY_INITIALIZED;

    if (NumberOfEnumeratedDevices < 0 || update) {
        // First enumeration or update
        int i;
        unsigned int j, sum;
        cmnGenericObject* go;
        svlVidCapSrcBase* api;
        DeviceInfo **apideviceinfos = new DeviceInfo*[NumberOfSupportedAPIs];
        int *apidevicecounts = new int[NumberOfSupportedAPIs];
        ImageFormat ***apiformats = new ImageFormat**[NumberOfSupportedAPIs];
        int **apiformatcounts = new int*[NumberOfSupportedAPIs];

        // Release previous results, if any
        if (EnumeratedDevices) {
            delete [] EnumeratedDevices;
            EnumeratedDevices = 0;
        }
        if (FormatList) {
            for (i = 0; i < NumberOfEnumeratedDevices; i ++) {
                if (FormatList[i]) delete [] FormatList[i];
            }
            delete [] FormatList;
            FormatList = 0;
        }
        if (FormatListSize) {
            delete [] FormatListSize;
            FormatListSize = 0;
        }
        NumberOfEnumeratedDevices = 0;

        // Enumerate registered APIs and store all results
        for (j = 0; j < NumberOfSupportedAPIs; j ++) {

            // Most device objects can be created dynamically
            go = SupportedAPIs[j]->Create();

            if (go == 0) {
#if (CISST_SVL_HAS_MIL == ON)
                // MIL device object is a singleton, cannot be created dynamically
                if (APIPlatforms[j] == MatroxImaging) {
                    go = svlVidCapSrcMIL::GetInstance();
                }
#endif // CISST_SVL_HAS_MIL
            }

            api = dynamic_cast<svlVidCapSrcBase*>(go);

            if (api) {
                apideviceinfos[j] = 0;
                apidevicecounts[j] = api->GetDeviceList(&(apideviceinfos[j]));
                if (apidevicecounts[j] > 0) {
                    apiformats[j] = new ImageFormat*[apidevicecounts[j]];
                    apiformatcounts[j] = new int[apidevicecounts[j]];
                    for (i = 0; i < apidevicecounts[j]; i ++)
                        apiformatcounts[j][i] = api->GetFormatList(i, &(apiformats[j][i]));
                }
                if (apidevicecounts[j] > 0) NumberOfEnumeratedDevices += apidevicecounts[j];
            }

            // Delete method will release only dynamically created objects
            SupportedAPIs[j]->Delete(go);
        }

        if (NumberOfEnumeratedDevices > 0) {
            // Allocate the ONE device info array
            EnumeratedDevices = new DeviceInfo[NumberOfEnumeratedDevices];
            // Allocate and reset format array
            FormatList = new ImageFormat*[NumberOfEnumeratedDevices];
            FormatListSize = new int[NumberOfEnumeratedDevices];
            memset(FormatList, 0, NumberOfEnumeratedDevices * sizeof(ImageFormat*));
            memset(FormatListSize, 0, NumberOfEnumeratedDevices * sizeof(int));

            // Copy API specific device info arrays into the ONE array
            sum = 0;
            for (j = 0; j < NumberOfSupportedAPIs; j ++) {
                if (apideviceinfos[j] && apidevicecounts[j] > 0) {

                    // Store formats
                    for (i = 0; i < apidevicecounts[j]; i ++) {
                        if (apiformatcounts[j][i] > 0) {
                            FormatList[sum + i] = apiformats[j][i];
                            FormatListSize[sum + i] = apiformatcounts[j][i];
                        }
                    }
                    delete [] apiformats[j];
                    delete [] apiformatcounts[j];

                    // Store device infos
                    memcpy(&(EnumeratedDevices[sum]), apideviceinfos[j], apidevicecounts[j] * sizeof(DeviceInfo));

                    sum += apidevicecounts[j];
                }
            }
        }

        // Free up allocated resources
        for (j = 0; j < NumberOfSupportedAPIs; j ++) {
            if (apideviceinfos[j]) delete [] apideviceinfos[j];
        }
        delete [] apideviceinfos;
        delete [] apidevicecounts;
        delete [] apiformats;
        delete [] apiformatcounts;
    }

    if (deviceinfolist && NumberOfEnumeratedDevices > 0) {
        deviceinfolist[0] = new DeviceInfo[NumberOfEnumeratedDevices];
        // The caller is responsible for deleting the deviceinfolist array after calling
        memcpy(deviceinfolist[0], EnumeratedDevices, NumberOfEnumeratedDevices * sizeof(DeviceInfo));
    }

    return NumberOfEnumeratedDevices;
}

void svlFilterSourceVideoCapture::ReleaseDeviceList(DeviceInfo *deviceinfolist)
{
    if (deviceinfolist) delete [] deviceinfolist;
}

int svlFilterSourceVideoCapture::PrintDeviceList(bool update)
{
    if (OutputImage == 0)
        return SVL_FAIL;

    DeviceInfo *devices = 0;
    int devicecount = GetDeviceList(&devices, update);

    for (int i = 0; i < devicecount; i ++) {
        std::cout << " " << i << ") " << devices[i].name << std::endl;
    }

    ReleaseDeviceList(devices);

    return devicecount;
}

int svlFilterSourceVideoCapture::PrintInputList(int deviceid, bool update)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    
    DeviceInfo *devices = 0;
    int devicecount = GetDeviceList(&devices, update);

    if (deviceid >= devicecount) {
        ReleaseDeviceList(devices);
        return SVL_FAIL;
    }

    int inputcount = devices[deviceid].inputcount;
    for (int i = 0; i < inputcount; i ++) {
        std::cout << "  " << i << ") " << devices[deviceid].inputnames[i] << std::endl;
    }

    ReleaseDeviceList(devices);

    return inputcount;
}

int svlFilterSourceVideoCapture::SetDevice(int deviceid, int inputid, unsigned int videoch)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (IsInitialized() == true)
        return SVL_ALREADY_INITIALIZED;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;

    DeviceID[videoch] = deviceid;
    InputID[videoch] = inputid;

    return SVL_OK;
}

int svlFilterSourceVideoCapture::GetDevice(int & deviceid, int & inputid, unsigned int videoch)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;

    deviceid = DeviceID[videoch];
    inputid = InputID[videoch];

    return SVL_OK;
}

int svlFilterSourceVideoCapture::GetFormatList(ImageFormat **formatlist, unsigned int videoch)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (formatlist == 0)
        return SVL_FAIL;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;
    if (DeviceID[videoch] < 0)
        return SVL_VCS_UNABLE_TO_OPEN;

    // Make sure devices are enumerated
    GetDeviceList(0);

    if (FormatList[DeviceID[videoch]] == 0 || FormatListSize[DeviceID[videoch]] < 1) return SVL_FAIL;

    formatlist[0] = new ImageFormat[FormatListSize[DeviceID[videoch]]];
    memcpy(formatlist[0], FormatList[DeviceID[videoch]], FormatListSize[DeviceID[videoch]] * sizeof(ImageFormat));

    return FormatListSize[DeviceID[videoch]];
}

void svlFilterSourceVideoCapture::ReleaseFormatList(ImageFormat *formatlist)
{
    if (formatlist) delete [] formatlist;
}

int svlFilterSourceVideoCapture::PrintFormatList(unsigned int videoch)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    
    int i, j;
    ImageFormat *formats = 0;
    int formatcount = GetFormatList(&formats, videoch);

    for (i = 0; i < formatcount; i ++) {
        std::cout << "  " << i << ") " << formats[i].width << "x" << formats[i].height << " ";
        std::cout << GetPixelTypeName(formats[i].colorspace);
        if (formats[i].framerate > 0.0) {
            std::cout << " (<=" << formats[i].framerate << "fps)" << std::endl;
        }
        else {
            std::cout << " (unknown framerate)" << std::endl;
        }
        if (formats[i].custom_mode >= 0) {
            std::cout << "      [CUSTOM mode=" << formats[i].custom_mode << std::endl;
            std::cout << "              maxsize=(" << formats[i].custom_maxwidth << ", " << formats[i].custom_maxheight << "); ";
            std::cout << "unit=(" << formats[i].custom_unitwidth << ", " << formats[i].custom_unitheight << ")" << std::endl;
            std::cout << "              roipos=(" << formats[i].custom_roileft << ", " << formats[i].custom_roitop << "); ";
            std::cout << "unit=(" << formats[i].custom_unitleft << ", " << formats[i].custom_unittop << ")" << std::endl;
            std::cout << "              colorspaces=(";
            for (j = 0; j < PixelTypeCount && formats[i].custom_colorspaces[j] != PixelUnknown; j ++) {
                if (j > 0) std::cout << ", ";
                std::cout << GetPixelTypeName(formats[i].custom_colorspaces[j]);
            }
            std::cout << ")" << std::endl;
            std::cout << "              pattern=" << GetPatternTypeName(formats[i].custom_pattern) << "]" << std::endl;
        }
    }

    ReleaseFormatList(formats);

    return formatcount;
}

int svlFilterSourceVideoCapture::SelectFormat(unsigned int formatid, unsigned int videoch)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    
    ImageFormat *formats = 0;
    int formatcount = GetFormatList(&formats, videoch);
    int ret = SVL_FAIL;

    if (static_cast<int>(formatid) < formatcount) {
        ret = SetFormat(formats[formatid], videoch);
    }

    ReleaseFormatList(formats);

    return ret;
}

int svlFilterSourceVideoCapture::SetFormat(ImageFormat& format, unsigned int videoch)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (IsInitialized() == true)
        return SVL_ALREADY_INITIALIZED;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;

    if (Format[videoch] == 0) Format[videoch] = new ImageFormat;
    memcpy(Format[videoch], &format, sizeof(ImageFormat));

    return SVL_OK;
}

int svlFilterSourceVideoCapture::GetFormat(ImageFormat& format, unsigned int videoch)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;
    if (Format[videoch] == 0)
        return SVL_FAIL;

    memcpy(&format, Format[videoch], sizeof(ImageFormat));

    return SVL_OK;
}

int svlFilterSourceVideoCapture::SetTrigger(ExternalTrigger& trigger, unsigned int videoch)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (IsInitialized() == true)
        return SVL_ALREADY_INITIALIZED;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;

    memcpy(&(Trigger[videoch]), &trigger, sizeof(ExternalTrigger));

    return SVL_OK;
}

int svlFilterSourceVideoCapture::GetTrigger(ExternalTrigger& trigger, unsigned int videoch)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;

    memcpy(&trigger, &(Trigger[videoch]), sizeof(ExternalTrigger));

    return SVL_OK;
}

int svlFilterSourceVideoCapture::SetImageProperties(ImageProperties& properties, unsigned int videoch)
{
    // Available only after initialization
    if (IsInitialized() == false)
        return SVL_NOT_INITIALIZED;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;

    if (DeviceObj[API[videoch]]->SetImageProperties(properties, APIChannelID[videoch]) == SVL_OK) {
        if (Properties[videoch] == 0) Properties[videoch] = new ImageProperties;
        memcpy(Properties[videoch], &properties, sizeof(ImageProperties));
        Properties[videoch]->mask = propShutter & propGain & propWhiteBalance & propBrightness & propGamma & propSaturation;
        return SVL_OK;
    }

    return SVL_FAIL;
}

int svlFilterSourceVideoCapture::SetImageProperties(unsigned int videoch)
{
    // Available only after initialization
    if (IsInitialized() == false)
        return SVL_NOT_INITIALIZED;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;
    if (Properties[videoch] == 0)
        return SVL_FAIL;

    return DeviceObj[API[videoch]]->SetImageProperties(Properties[videoch][0], APIChannelID[videoch]);
}

int svlFilterSourceVideoCapture::GetImageProperties(ImageProperties& properties, unsigned int videoch)
{
    // Available only after initialization
    if (IsInitialized() == false)
        return SVL_NOT_INITIALIZED;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;

    if (DeviceObj[API[videoch]]->GetImageProperties(properties, APIChannelID[videoch]) != SVL_OK) return SVL_FAIL;

    if (Properties[videoch] == 0) Properties[videoch] = new ImageProperties;
    memcpy(Properties[videoch], &properties, sizeof(ImageProperties));
    Properties[videoch]->mask = propShutter & propGain & propWhiteBalance & propBrightness & propGamma & propSaturation;

    return SVL_OK;
}

int svlFilterSourceVideoCapture::GetImageProperties(unsigned int videoch)
{
    // Available only after initialization
    if (IsInitialized() == false)
        return SVL_NOT_INITIALIZED;
    if (videoch >= NumberOfChannels)
        return SVL_WRONG_CHANNEL;

    if (Properties[videoch] == 0) Properties[videoch] = new ImageProperties;
    Properties[videoch]->mask = propShutter & propGain & propWhiteBalance & propBrightness & propGamma & propSaturation;

    return DeviceObj[API[videoch]]->GetImageProperties(Properties[videoch][0], APIChannelID[videoch]);
}

std::string svlFilterSourceVideoCapture::GetPixelTypeName(PixelType pixeltype)
{
    switch (pixeltype) {
        case PixelRAW8:     return "RAW8";
        case PixelRAW16:    return "RAW16";
        case PixelRGB8:     return "RGB24";
        case PixelYUV444:   return "YUV444";
        case PixelYUV422:   return "YUV422";
        case PixelYUV411:   return "YUV411";
        case PixelMONO8:    return "Mono8";
        case PixelMONO16:   return "Mono16";
        case PixelUnknown:
        default:            return "Unknown color space";
    }
}

std::string svlFilterSourceVideoCapture::GetPatternTypeName(PatternType patterntype)
{
    switch (patterntype) {
        case PatternRGGB:   return "RGGB";
        case PatternGBRG:   return "GBRG";
        case PatternGRBG:   return "GRBG";
        case PatternBGGR:   return "BGGR";
        case PatternUnknown:
        default:            return "Unknown pattern";
    }
}

int svlFilterSourceVideoCapture::SaveSettings(const char* filepath)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    
    unsigned int writelen;
    int err, devid, intvalue;
    unsigned char emptybuffer[SVL_VCS_STRING_LENGTH];
	FILE *fp;

    memset(emptybuffer, 0, SVL_VCS_STRING_LENGTH);
    err = SVL_FAIL;

    fp = fopen(filepath, "wb");
    if (fp == NULL) goto labError;

	// Write "number of channels"
    writelen = static_cast<unsigned int>(fwrite(&NumberOfChannels, sizeof(unsigned int), 1, fp));
	if (writelen < 1) goto labError;

    // Make sure devices are enumerated
    GetDeviceList(0);

    // For each channel
    for (unsigned int i = 0; i < NumberOfChannels; i ++) {

		// Write "device id"
        devid = DeviceID[i];
        // Check if device ID is in range
        if (devid >= NumberOfEnumeratedDevices) devid = -1;
        writelen = static_cast<unsigned int>(fwrite(&devid, sizeof(int), 1, fp));
		if (writelen < 1) goto labError;

        // Write "device name"
        if (devid >= 0) writelen = static_cast<unsigned int>(fwrite(EnumeratedDevices[DeviceID[i]].name, SVL_VCS_STRING_LENGTH, 1, fp));
        else writelen = static_cast<unsigned int>(fwrite(emptybuffer, SVL_VCS_STRING_LENGTH, 1, fp));
		if (writelen < 1) goto labError;

        // Write "input id"
        intvalue = InputID[i];
        writelen = static_cast<unsigned int>(fwrite(&intvalue, sizeof(int), 1, fp));
		if (writelen < 1) goto labError;

        // Write "format size"
        if (Format[i]) intvalue = sizeof(ImageFormat);
        else intvalue = 0;
        writelen = static_cast<unsigned int>(fwrite(&intvalue, sizeof(int), 1, fp));
		if (writelen < 1) goto labError;

        // Write "format"
        if (intvalue > 0) {
            writelen = static_cast<unsigned int>(fwrite(Format[i], 1, sizeof(ImageFormat), fp));
		    if (writelen < sizeof(ImageFormat)) goto labError;
	    }

        // Write "properties size"
        if (Properties[i]) intvalue = sizeof(ImageProperties);
        else intvalue = 0;
        writelen = static_cast<unsigned int>(fwrite(&intvalue, sizeof(int), 1, fp));
		if (writelen < 1) goto labError;

        // Write "properties"
        if (intvalue > 0) {
            writelen = static_cast<unsigned int>(fwrite(Properties[i], 1, sizeof(ImageProperties), fp));
		    if (writelen < sizeof(ImageProperties)) goto labError;
	    }

        // Write "trigger size"
        intvalue = sizeof(ExternalTrigger);
        writelen = static_cast<unsigned int>(fwrite(&intvalue, sizeof(int), 1, fp));
		if (writelen < 1) goto labError;

        // Write "trigger"
        if (intvalue > 0) {
            writelen = static_cast<unsigned int>(fwrite(&(Trigger[i]), 1, sizeof(ExternalTrigger), fp));
		    if (writelen < sizeof(ExternalTrigger)) goto labError;
	    }

        // Write "device specific configuration buffer size"
        writelen = static_cast<unsigned int>(fwrite(&(DevSpecConfigBufferSize[i]), sizeof(unsigned int), 1, fp));
		if (writelen < 1) goto labError;

        // Write "device specific configuration buffer"
        writelen = static_cast<unsigned int>(fwrite(DevSpecConfigBuffer[i], 1, DevSpecConfigBufferSize[i], fp));
		if (writelen < DevSpecConfigBufferSize[i]) goto labError;
    }

    err = SVL_OK;

labError:
    if (fp != NULL) fclose(fp);

    return err;
}

int svlFilterSourceVideoCapture::LoadSettings(const char* filepath)
{
    if (OutputImage == 0)
        return SVL_FAIL;
    if (IsInitialized() == true)
        return SVL_ALREADY_INITIALIZED;

    int intvalue;
    unsigned int i, readlen;
    unsigned int uintvalue;
    char buffer[SVL_VCS_STRING_LENGTH];
	FILE *fp;

    fp = fopen(filepath, "rb");
    if (fp == 0) return SVL_FAIL;

    // Read "number of channels"
    readlen = static_cast<unsigned int>(fread(&uintvalue, sizeof(unsigned int), 1, fp));
	if (readlen < 1 || uintvalue != NumberOfChannels) {
        fclose(fp);
        return SVL_FAIL;
    }

    // Make sure devices are enumerated
    GetDeviceList(0);

    // For each channel
    for (i = 0; i < NumberOfChannels; i ++) {

		// Read "device id"
        readlen = static_cast<unsigned int>(fread(&intvalue, sizeof(int), 1, fp));
		if (readlen < 1) goto labError;
        if (intvalue < -1 || intvalue >= NumberOfEnumeratedDevices) goto labError;
        DeviceID[i] = intvalue;

        // Read "device name"
        readlen = static_cast<unsigned int>(fread(buffer, SVL_VCS_STRING_LENGTH, 1, fp));
		if (readlen < 1) goto labError;
        if (DeviceID[i] >= 0) {
            buffer[SVL_VCS_STRING_LENGTH - 1] = 0;
            if (strcmp(EnumeratedDevices[DeviceID[i]].name, buffer) != 0) goto labError;
        }

        // Read "input id"
        readlen = static_cast<unsigned int>(fread(&intvalue, sizeof(int), 1, fp));
		if (readlen < 1) goto labError;
        if (DeviceID[i] >= 0) InputID[i] = intvalue;
        else InputID[i] = -1;

        // Read "format size"
        readlen = static_cast<unsigned int>(fread(&intvalue, sizeof(int), 1, fp));
		if (readlen < 1) goto labError;

        // Read "format"
        if (intvalue == sizeof(ImageFormat)) {
            if (Format[i] == 0) Format[i] = new ImageFormat;
            readlen = static_cast<unsigned int>(fread(Format[i], 1, sizeof(ImageFormat), fp));
            if (readlen < sizeof(ImageFormat)) {
                delete Format[i];
                Format[i] = 0;
                goto labError;
            }
        }

        // Read "properties size"
        readlen = static_cast<unsigned int>(fread(&intvalue, sizeof(int), 1, fp));
		if (readlen < 1) goto labError;

        // Read "properties"
        if (intvalue == sizeof(ImageProperties)) {
            if (Properties[i] == 0) Properties[i] = new ImageProperties;
            readlen = static_cast<unsigned int>(fread(Properties[i], 1, sizeof(ImageProperties), fp));
            if (readlen < sizeof(ImageProperties)) {
                delete Properties[i];
                Properties[i] = 0;
                goto labError;
            }
        }

        // Read "trigger size"
        readlen = static_cast<unsigned int>(fread(&intvalue, sizeof(int), 1, fp));
		if (readlen < 1) goto labError;

        // Read "trigger"
        if (intvalue == sizeof(ExternalTrigger)) {
            readlen = static_cast<unsigned int>(fread(&(Trigger[i]), 1, sizeof(ExternalTrigger), fp));
            if (readlen < sizeof(ExternalTrigger)) {
                memset(&(Trigger[i]), 0, sizeof(ExternalTrigger));
                goto labError;
            }
        }

        // Read "device specific configuration buffer size"
        readlen = static_cast<unsigned int>(fread(&uintvalue, sizeof(unsigned int), 1, fp));
		if (readlen < 1) goto labError;
        if (DeviceID[i] >= 0) {
            if (uintvalue >= MAX_PROPERTIES_BUFFER_SIZE) goto labError;
            DevSpecConfigBufferSize[i] = uintvalue;
        }
        else DevSpecConfigBufferSize[i] = 0;

        // Read "device specific configuration buffer"
        if (DevSpecConfigBuffer[i]) delete [] DevSpecConfigBuffer[i];
        DevSpecConfigBuffer[i] = 0;
        if (uintvalue > 0) {
            DevSpecConfigBuffer[i] = new unsigned char[uintvalue];
            readlen = static_cast<unsigned int>(fread(DevSpecConfigBuffer[i], 1, uintvalue, fp));
    		if (readlen < uintvalue) goto labError;
        }
    }

    fclose(fp);
    return SVL_OK;

labError:
    for (i = 0; i < NumberOfChannels; i ++) {
        DeviceID[i] = InputID[i] = -1;
        if (DevSpecConfigBuffer[i]) delete [] DevSpecConfigBuffer[i];
        DevSpecConfigBuffer[i] = 0;
        DevSpecConfigBufferSize[i] = 0;
    }
    fclose(fp);
    return SVL_FAIL;
}


/***************************************/
/*** svlVidCapSrcBase class ************/
/***************************************/

int svlVidCapSrcBase::GetFormatList(unsigned int CMN_UNUSED(deviceid), svlFilterSourceVideoCapture::ImageFormat ** CMN_UNUSED(formatlist))
{
    return SVL_FAIL;
}

int svlVidCapSrcBase::SetFormat(svlFilterSourceVideoCapture::ImageFormat & CMN_UNUSED(format), unsigned int CMN_UNUSED(videoch))
{
    return SVL_FAIL;
}

int svlVidCapSrcBase::GetFormat(svlFilterSourceVideoCapture::ImageFormat & CMN_UNUSED(format), unsigned int CMN_UNUSED(videoch))
{
    return SVL_FAIL;
}

int svlVidCapSrcBase::SetImageProperties(svlFilterSourceVideoCapture::ImageProperties & CMN_UNUSED(properties), unsigned int CMN_UNUSED(videoch))
{
    return SVL_FAIL;
}

int svlVidCapSrcBase::GetImageProperties(svlFilterSourceVideoCapture::ImageProperties & CMN_UNUSED(properties), unsigned int CMN_UNUSED(videoch))
{
    return SVL_FAIL;
}

int svlVidCapSrcBase::SetTrigger(svlFilterSourceVideoCapture::ExternalTrigger & CMN_UNUSED(trigger), unsigned int CMN_UNUSED(videoch))
{
    return SVL_FAIL;
}

int svlVidCapSrcBase::GetTrigger(svlFilterSourceVideoCapture::ExternalTrigger & CMN_UNUSED(trigger), unsigned int CMN_UNUSED(videoch))
{
    return SVL_FAIL;
}


/***********************************************/
/*** svlVidCapSrcDialogThread class ************/
/***********************************************/

void* svlVidCapSrcDialogThread::Proc(svlFilterSourceVideoCapture* baseref)
{
    // signal success to main thread
    InitSuccess = true;
    InitEvent.Raise();

    svlFilterSourceVideoCapture::ImageProperties properties;

    // Create an empty line under the table
    printf("\r\n\x1b[1A");

    while (!Stopped) {

        if (baseref->GetImageProperties(properties, VideoChannel) == SVL_OK) {

            if (properties.manual & svlFilterSourceVideoCapture::propShutter) {
                printf("\r  |  %7d |", properties.shutter);
            }
            else {
                printf("\r  |  %6da |", properties.shutter);
            }
            if (properties.manual & svlFilterSourceVideoCapture::propGain) {
                printf("  %6d |", properties.gain);
            }
            else {
                printf("  %5da |", properties.gain);
            }
            if (properties.manual & svlFilterSourceVideoCapture::propWhiteBalance) {
                printf("  %5d | %6d |", properties.wb_u_b, properties.wb_v_r);
            }
            else {
                printf("  %4da | %5da |", properties.wb_u_b, properties.wb_v_r);
            }
            if (properties.manual & svlFilterSourceVideoCapture::propBrightness) {
                printf("  %9d |", properties.shutter);
            }
            else {
                printf("  %8da |", properties.shutter);
            }
            if (properties.manual & svlFilterSourceVideoCapture::propGamma) {
                printf("  %7d |", properties.gamma);
            }
            else {
                printf("  %6da |", properties.gamma);
            }
            if (properties.manual & svlFilterSourceVideoCapture::propSaturation) {
                printf("  %7d |", properties.saturation);
            }
            else {
                printf("  %6da |", properties.saturation);
            }
        }
        else {
            printf("\r  |      Image property adjustments not available. Press 'q' to return.     |");
        }

        fflush(stdout);
        osaSleep(0.1);
    }

	return this;
}

