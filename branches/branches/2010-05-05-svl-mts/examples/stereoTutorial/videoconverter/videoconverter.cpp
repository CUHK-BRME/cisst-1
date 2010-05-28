/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$
  
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


#include <cisstStereoVision.h>

using namespace std;


///////////////////////
//  Video Converter  //
///////////////////////

int VideoConverter(std::string &src_path, std::string &dst_path, bool loadcodec)
{
    svlInitialize();

    svlStreamManager stream(8);
    svlFilterSourceVideoFile source(1);
    svlFilterVideoFileWriter writer;

    if (src_path.empty()) {
        if (source.DialogFilePath() != SVL_OK) {
            cerr << " -!- No source file has been selected." << endl;
            return -1;
        }
        source.GetFilePath(src_path);
    }
    else {
        source.SetFilePath(src_path);
    }

    source.SetTargetFrequency(1000.0); // as fast as possible
    source.SetLoop(false);

    if (dst_path.empty()) {
        if (writer.DialogFilePath() != SVL_OK) {
            cerr << " -!- No destination file has been selected." << endl;
            return -1;
        }
        writer.GetFilePath(dst_path);
    }
    else {
        writer.SetFilePath(dst_path);
    }

    if (loadcodec) {
        if (writer.LoadCodec("codec.dat") != SVL_OK) {
            if (writer.DialogCodec() != SVL_OK) {
                cerr << " -!- Unable to set up compression." << endl;
                return -1;
            }
            writer.SaveCodec("codec.dat");
        }
    }
    else {
        if (writer.DialogCodec() != SVL_OK) {
            cerr << " -!- Unable to set up compression." << endl;
            return -1;
        }
    }

    std::string encoder;
    writer.GetCodecName(encoder);

    // chain filters to pipeline
    stream.SetSourceFilter(&source);
    source.GetOutput()->Connect(writer.GetInput());

    cerr << "Converting: '" << src_path << "' to '" << dst_path <<"' using codec: '" << encoder << "'" << endl;

    // initialize and start stream
    if (stream.Start() != SVL_OK) goto labError;

    do {
        cerr << " > Frames processed: " << source.GetFrameCounter() << "     \r";
    } while (stream.IsRunning() && stream.WaitForStop(0.5) == SVL_WAIT_TIMEOUT);
    cerr << " > Frames processed: " << source.GetFrameCounter() << "           " << endl;

    if (stream.GetStreamStatus() < 0) {
        // Some error
        cerr << " -!- Error occured during conversion." << endl;
    }
    else {
        // Success
        cerr << " > Conversion done." << endl;
    }

    // release pipeline
    stream.Release();

labError:
    return 0;
}


//////////////////////////////////
//             main             //
//////////////////////////////////

int main(int argc, char** argv)
{
    string source, destination;
    if (argc >= 3) destination = argv[2];
    if (argc >= 2) source = argv[1];
    else {
        cerr << endl << "stereoTutorialVideoConverter - cisstStereoVision example by Balazs Vagvolgyi" << endl;
        cerr << "See http://www.cisst.org/cisst for details." << endl;
        cerr << "Command line format:" << endl;
        cerr << "     stereoTutorialVideoConverter [source_pathname [destination_pathname]]" << endl;
        cerr << "Examples:" << endl;
        cerr << "     stereoTutorialVideoConverter" << endl;
        cerr << "     stereoTutorialVideoConverter src.cvi" << endl;
        cerr << "     stereoTutorialVideoConverter src.avi dest.cvi" << endl << endl;
    }

    VideoConverter(source, destination, true);

    return 1;
}

