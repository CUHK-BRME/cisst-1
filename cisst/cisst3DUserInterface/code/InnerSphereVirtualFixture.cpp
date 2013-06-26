/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
$Id: InnerSphereVirtualFixture.cpp 3148 2013-06-26 15:46:31Z oozgune1 $

Author(s):	Orhan Ozguner
Created on:	2013-06-26

(C) Copyright 2013 Johns Hopkins University (JHU), All Rights
Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/
#include <cisst3DUserInterface/InnerSphereVirtualFixture.h>"
#include <cisstVector.h>
#include <iostream>
#include <conio.h>
#include <cstdio>
#include <assert.h>
#include <ctype.h>
#include <fcntl.h>
#include<windows.h>


void InnerSphereVirtualFixture::findOrthogonal(vct3 in, vct3 &out1, vct3 &out2){
    vct3 vec1, vec2;
    vct3 axisY(0.0 , 1.0 , 0.0);
    vct3 axizZ(0.0 , 0.0 , 1.0);
    double len1, len2, len3;

    // Find 1st orthogonal vector
    len1 = in.Norm(); 
    in = in.Divide(len1); 
    // Use Y-axis unit vector to find first orthogonal
    vec1.CrossProductOf(in,axisY); 
    // Check to make sure the Y-axis unit vector is not too close to input unit vector,
    // if they are close dot product will be large and then use different arbitrary unit vector
    if ( vctDotProduct(in, vec1) >= 0.98){
        vec1.CrossProductOf(in,axizZ); 
        std::cout<<"Something is not good..."<<std::endl;
    }
    // Now find 2nd orthogonal vector
    vec2.CrossProductOf(in,vec1); 
    len2 = vec1.Norm(); 
    len3 = vec2.Norm(); 

    out1 = vec1.Divide(len2);
    out2 = vec2.Divide(len3);
}

void InnerSphereVirtualFixture::update(const vctFrm3 & pos ,prmFixtureGainCartesianSet & vfParams) {
    vct3 position;  //<! final force position
    vctMatRot3 rotation;  //<! final force orientation
    vct3 pos_error; //<! position error between current and center position
    vct3 currentPosition; //<! current MTM position
    vct3 norm_vector; //<! normal vector 
    vct3 scaled_norm_vector; //<! norm vector scaled with -radius
    vct3 ortho1(0.0); //<! orthogonal vector to the norm vector
    vct3 ortho2(0.0); //<! orthogonal vector to the norm vector
    double distance; //<! distance between current position and center position
    vct3 stiffnessPos; //<! positive position stiffness

    //get curent position
    currentPosition = pos.Translation();
    //calculate position error
    pos_error = (center-currentPosition);
    //find distance (norm of pos_error)
    distance = pos_error.Norm();
    //scale pos_error to calculate norm vector
    norm_vector = pos_error.Divide(distance);
    //find 2 orthogonal vectors to the norm vector
    findOrthogonal(norm_vector,ortho1,ortho2);
    //form rotation using orthogonal vectors
    rotation.Column(0).Assign(ortho2);
    rotation.Column(1).Assign(-ortho1);
    rotation.Column(2).Assign(norm_vector);
    //scale norm vector with radius
    scaled_norm_vector = norm_vector.Multiply(-radius);
    //add scaled_norm_vector to the sphere center
    position = center+scaled_norm_vector;

    //set force position
    vfParams.SetForcePosition(position);
    //set force orientation
    vfParams.SetForceOrientation(rotation);
    //set torque orientation 
    vfParams.SetTorqueOrientation(rotation);
    //set Position Stiffness
    stiffnessPos.SetAll(0.0);
    stiffnessPos.Z() = -500.0;
    vfParams.SetPositionStiffnessPos(stiffnessPos);

    //Temporary hard code solution ask Anton for better way
    vct3 temp;
    temp.SetAll(0.0);
    vfParams.SetPositionStiffnessNeg(temp);
    vfParams.SetPositionDampingPos(temp);
    vfParams.SetPositionDampingNeg(temp);
    vfParams.SetForceBiasPos(temp);
    vfParams.SetForceBiasNeg(temp);
    vfParams.SetOrientationStiffnessPos(temp);
    vfParams.SetOrientationStiffnessNeg(temp);
    vfParams.SetOrientationDampingPos(temp);
    vfParams.SetOrientationDampingNeg(temp);
    vfParams.SetTorqueBiasPos(temp);
    vfParams.SetTorqueBiasNeg(temp);

}

void InnerSphereVirtualFixture::setCenter(const vct3 & c){
    center = c;
}
void InnerSphereVirtualFixture::setRadius(const double & r){
    radius = r;
}

double InnerSphereVirtualFixture::getRadius(void){
    return radius;
}

vct3 InnerSphereVirtualFixture::getCenter(void){
    return center;
}
