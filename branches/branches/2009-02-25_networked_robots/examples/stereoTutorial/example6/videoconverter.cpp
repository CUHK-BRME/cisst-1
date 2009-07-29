/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: $
  
  Author(s):  Balazs Vagvolgyi
  Created on: 2009

  (C) Copyright 2006-2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/


#ifdef _WIN32
#include <conio.h>
#endif // _WIN32

#include <iostream>
#include <string>
#include <cisstStereoVision.h>

#ifdef __GNUC__
#include <curses.h>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#endif // __GNUC__

using namespace std;


///////////////////////
//  Video Converter  //
///////////////////////

int VideoConverter(const std::string source, const std::string destination)
{
    // instantiating SVL stream and filters
    svlStreamManager viewer_stream(1);
    svlVideoFileSource viewer_source(true);
    svlImageRectifier viewer_rectifier;
    svlImageFileWriter viewer_writer;
    svlImageWindow viewer_window;

    // setup source
    viewer_source.DialogFilePath(SVL_LEFT);
    viewer_source.DialogFilePath(SVL_RIGHT);

    // setup source
    viewer_rectifier.LoadTable("C:\\Documents and Settings\\bvagvolgyi\\My Documents\\Calibration\\20071128_JHMI_daVinci_0_degree\\left_rectif.txt", SVL_LEFT);
    viewer_rectifier.LoadTable("C:\\Documents and Settings\\bvagvolgyi\\My Documents\\Calibration\\20071128_JHMI_daVinci_0_degree\\right_rectif.txt", SVL_RIGHT);

    // setup writer
    viewer_writer.SetFilePath("left_", "bmp", SVL_LEFT);
    viewer_writer.SetFilePath("right_", "bmp", SVL_RIGHT);

    // setup image window
    viewer_window.SetTitleText("Video Player");

    // chain filters to pipeline
    viewer_stream.Trunk().Append(&viewer_source);
    viewer_stream.Trunk().Append(&viewer_rectifier);
    viewer_stream.Trunk().Append(&viewer_writer);
    viewer_stream.Trunk().Append(&viewer_window);

    cerr << endl << "Starting stream... ";

    // initialize and start stream
    if (viewer_stream.Start() != SVL_OK) goto labError;

    cerr << "Done" << endl;

#ifdef __GNUC__
    ////////////////////////////////////////////////////
    // modify terminal settings for single key inputs
    struct  termios ksettings;
    struct  termios new_ksettings;
    int     kbrd;
    kbrd = open("/dev/tty",O_RDWR);
    
    #if (CISST_OS == CISST_LINUX)
        ioctl(kbrd, TCGETS, &ksettings);
        new_ksettings = ksettings;
        new_ksettings.c_lflag &= !ICANON;
        new_ksettings.c_lflag &= !ECHO;
        ioctl(kbrd, TCSETS, &new_ksettings);
        ioctl(kbrd, TIOCNOTTY);
    #endif // (CISST_OS == CISST_LINUX)
    #if (CISST_OS == CISST_DARWIN)
        ioctl(kbrd, TIOCGETA, &ksettings);
        new_ksettings = ksettings;
        new_ksettings.c_lflag &= !ICANON;
        new_ksettings.c_lflag &= !ECHO;
        ioctl(kbrd, TIOCSETA, &new_ksettings);
        ////////////////////////////////////////////////////
    #endif // (CISST_OS == CISST_DARWIN)
#endif

    // wait for keyboard input in command window
#ifdef _WIN32
    int ch;
#endif
#ifdef __GNUC__
    char ch;
#endif

    do {
        cerr << endl << "Keyboard commands:" << endl << endl;
        cerr << "  In command window:" << endl;
        cerr << "    'q'   - Quit" << endl << endl;

#ifdef _WIN32
        ch = _getch();
#endif
#ifdef __GNUC__
        ch = getchar();
#endif
    } while (ch != 'q');

#ifdef __GNUC__
    ////////////////////////////////////////////////////
    // reset terminal settings    
    #if (CISST_OS == CISST_LINUX)
        ioctl(kbrd, TCSETS, &ksettings);
    #endif // (CISST_OS == CISST_LINUX)
    #if (CISST_OS == CISST_DARWIN)
        ioctl(kbrd, TIOCSETA, &ksettings);
    #endif // (CISST_OS == CISST_DARWIN)
    
    close(kbrd);
    ////////////////////////////////////////////////////
#endif

    cerr << endl;

    // stop stream
    viewer_stream.Stop();

    // destroy pipeline
    viewer_stream.EmptyFilterList();

labError:
    return 0;
}


//////////////////////////////////
//             main             //
//////////////////////////////////

int ParseNumber(char* string, unsigned int maxlen)
{
    if (string == 0 || maxlen == 0) return -1;

    int ivalue, j;
    char ch;

    // parse number
    j = 0;
    ivalue = 0;
    ch = string[j];
    // 4 digits max
    while (ch != 0 && j < (int)maxlen) {
        // check if number
        ch -= '0';
        if (ch > 9 || ch < 0) {
            ivalue = -1;
            break;
        }
        ivalue = ivalue * 10 + ch;
        // step to next digit
        j ++;
        ch = string[j];
    }
    if (j == 0) ivalue = -1;

    return ivalue;
}

int main(int argc, char** argv)
{
    cerr << endl << "stereoTutorialVideoConverter - cisstStereoVision example by Balazs Vagvolgyi" << endl;
    cerr << "See http://www.cisst.org/cisst for details." << endl << endl;

    if (argc < 1/*3*/) {
        cerr << "Command line format:" << endl;
        cerr << "     stereoTutorialVideoConverter <source_pathname>  <destination_pathname>" << endl;
        cerr << "Example:" << endl;
        cerr << "     stereoTutorialVideoConverter video.avi video.cvi" << endl;
        cerr << "     stereoTutorialVideoConverter video.cvi video.avi" << endl;
        cerr << "Quit" << endl << endl;
        return 1;
    }

    VideoConverter("", "");//argv[1], argv[2]);

    cerr << "Quit" << endl << endl;
    return 1;
}

