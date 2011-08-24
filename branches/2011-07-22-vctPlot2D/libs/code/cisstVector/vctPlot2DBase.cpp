/* -*- Mode: C++; wtab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: vctPlot2DBase.cpp 1238 2010-02-27 03:16:01Z auneri1 $

  Author(s):  Anton Deguet
  Created on: 2010-05-05

  (C) Copyright 2010-2011 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.


--- end cisst license ---
*/


#include <cisstVector/vctPlot2DBase.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnAssert.h>
#include <iostream>
#include <string>


/***************************************************************************************************
    Scale Functions
***************************************************************************************************/


vctPlot2DBase::Scale::Scale(const std::string & name, size_t pointDimension){
    this->PointSize = pointDimension;
    this->Translation.SetAll(0.0);
    this->ScaleValue.SetAll(1.0);
    this->Name = name;
    SetContinuousFitX(true);
    SetContinuousFitY(true);

}

vctPlot2DBase::Scale::~Scale(){


}

std::string vctPlot2DBase::Scale::GetName(void){
    
    return this->Name;
}

void vctPlot2DBase::Scale::FitX(double padding)
{
    double min, max;
    this->ComputeDataRangeX(min, max);
    this->FitX(min, max, padding);
}


void vctPlot2DBase::Scale::FitX(double min, double max, double padding)
{
    this->ViewingRangeX.Assign(min, max);
    this->ScaleValue.X() = this->Viewport.X() / ((max - min) * (1.0 + padding));
    this->Translation.X() =
        - min * this->ScaleValue.X()
        + 0.5 * padding * this->Viewport.X();
}

void vctPlot2DBase::Scale::FitY(double padding)
{
    double min, max;
    this->ComputeDataRangeY(min, max);
    this->FitY(min, max, padding);
}


void vctPlot2DBase::Scale::FitY(double min, double max, double padding)
{
    this->ViewingRangeY.Assign(min, max);
    this->ScaleValue.Y() = this->Viewport.Y() / ((max - min) * (1.0 + padding));
    this->Translation.Y() =
        - min * this->ScaleValue.Y()
        + 0.5 * padding * this->Viewport.Y();
}

void vctPlot2DBase::Scale::FitXY(const vctDouble2 & padding)
{
    vctDouble2 min, max;
    this->ComputeDataRangeXY(min, max);
    this->FitXY(min, max, padding);
}


void vctPlot2DBase::Scale::FitXY(vctDouble2 min, vctDouble2 max, const vctDouble2 & padding)
{
    this->ViewingRangeX.Assign(min.X(), max.X());
    this->ViewingRangeY.Assign(min.Y(), max.Y());
    // compute scale
    vctDouble2 dataDiff;
    dataDiff.DifferenceOf(max, min);
    this->ScaleValue.ElementwiseRatioOf(this->Viewport, dataDiff);
    vctDouble2 pad(padding);
    pad.Add(1.0);
    this->ScaleValue.ElementwiseDivide(pad);
    // compute translation
    this->Translation.ElementwiseProductOf(min, this->ScaleValue);
    this->Translation.NegationSelf();
    pad.ElementwiseProductOf(padding, this->Viewport);
    pad.Multiply(0.5);
    this->Translation.Add(pad);

}

// call each trace and find the min & max 
bool vctPlot2DBase::Scale::ComputeDataRangeX(double & min, double & max, bool assumesDataSorted) const{

    size_t traceIndex;
    const size_t numberofTraces = this->Traces.size();
    double rmin, rmax;


    if(numberofTraces != 0){
        min = this->Traces[0]->Data.Element(this->Traces[0]->IndexFirst).X();
        rmin = rmax = max = min;
    }else{
        min = -1;
        max = 1;
        return false;
    }

    for(traceIndex = 0; traceIndex < numberofTraces; traceIndex++){
        vctPlot2DBase::Trace *trace = this->Traces[traceIndex];
        trace->ComputeDataRangeX(min, max, assumesDataSorted);
        // get Scale[min,max]
        rmin = (rmin > min) ? min : rmin;
        rmax = (rmax < max) ? max : rmax;
    }
    min = rmin;
    max = rmax;
    return true;
}

bool vctPlot2DBase::Scale::ComputeDataRangeY(double & min, double & max){
    size_t traceIndex;
    const size_t numberofTraces = this->Traces.size();
    double rmin, rmax;

    if(numberofTraces != 0){
        min = this->Traces[0]->Data.Element(this->Traces[0]->IndexFirst).Y();
        rmin = rmax = max = min;
    }else{
        min = -1;
        max = 1;
        return false;
    }
       
    for(traceIndex = 0; traceIndex < numberofTraces; traceIndex++){
        vctPlot2DBase::Trace *trace = this->Traces[traceIndex];
        trace->ComputeDataRangeY(min, max);
        // get Scale[min,max]
        rmin = (rmin > min) ? min : rmin;
        rmax = (rmax < max) ? max : rmax;
    }
    min = rmin;
    max = rmax;
    return true;
}

bool vctPlot2DBase::Scale::ComputeDataRangeXY(vctDouble2 & min, vctDouble2 & max){
    size_t traceIndex;
    const size_t numberofTraces = this->Traces.size();
    vctDouble2 rmin, rmax;

    if(numberofTraces != 0){
        min = this->Traces[0]->Data.Element(this->Traces[0]->IndexFirst);
        rmin = rmax = max = min;
    }else{
        min.SetAll(-1.0);
        max.SetAll(1.0);
        return false;
    }
      
    for(traceIndex = 0; traceIndex < numberofTraces; traceIndex++){
        vctPlot2DBase::Trace *trace = this->Traces[traceIndex];
        trace->ComputeDataRangeXY(min, max);
        // get Scale[min,max]
        rmin.X() = (rmin.X() > min.X()) ? min.X():rmin.X();
        rmin.Y() = (rmin.Y() > min.Y()) ? min.Y():rmin.Y();
        
        rmax.X() = (rmax.X() < max.X()) ? max.X() : rmax.X();
        rmax.Y() = (rmax.Y() < max.Y()) ? max.Y() : rmax.Y();        
    }
    min = rmin;
    max = rmax;
    return true;
}


void vctPlot2DBase::Scale::SetContinuousFitX(bool fit){
    this->ContinuousFitX = fit;
    //    this->ContinuousAlignMaxX = false;
    this->Continuous = (this->ContinuousFitX
                        || this->ContinuousFitY);

}

void vctPlot2DBase::Scale::SetContinuousFitY(bool fit){
    this->ContinuousFitY = fit;
    this->Continuous = (this->ContinuousFitX
                        || this->ContinuousFitY);

}

void vctPlot2DBase::Scale::SetColor(const vctDouble3 & colorInRange0To1)
{
    vctDouble3 clippedColor;
    clippedColor.ClippedAboveOf(colorInRange0To1, 1.0);
    clippedColor.ClippedBelowOf(clippedColor, 0.0);
    this->Color.Assign(clippedColor);
}


vctPlot2DBase::Trace * vctPlot2DBase::Scale::AddSignal(const std::string & name){
    // check if the name already exists
    TracesIdType::const_iterator found = this->TracesId.find(name);
    const TracesIdType::const_iterator end = this->TracesId.end();
    std::cerr<<"outter addsignal: "<< Traces.size()<<"\n";
    if (found == end) {
        size_t SignalSize = Traces.size();
        // new name
        //set the number of points to default, 100
        Trace * newTrace = new Trace(name, 100, this->PointSize);
        newTrace->Parent = this;
        this->Traces.push_back(newTrace);
        TracesId[name] = SignalSize;
        std::cerr<<"add signal:" << name <<"\n";
        return newTrace;
    }
    return 0;
}

bool vctPlot2DBase::Scale::RemoveSignal(const std::string & name){
    // check if the name already exists
    TracesIdType::iterator found = this->TracesId.find(name);
    const TracesIdType::iterator end = this->TracesId.end();
    
    if(found != end){
        this->TracesId.erase(found);
    }
    
    
    for(size_t i = 0; i < Traces.size(); i++){
        if(name.find(Traces.at(i)->Name) != std::string::npos){
            this->Traces.erase(this->Traces.begin()+i);
            std::cerr<<"remove signal:" <<name<<"\n";
            return true;
        }
    }
    
    
    return false;
}

vctPlot2DBase::VerticalLine * vctPlot2DBase::Scale::AddVerticalLine(const std::string &name){    

    VerticalLinesIdType::const_iterator found = this->VerticalLinesId.find(name);
    const VerticalLinesIdType::const_iterator end = this->VerticalLinesId.end();

    if (found == end) {
        size_t vLineSize = VerticalLines.size();
        // new name
        //set the number of points to default, 100
        VerticalLine * newVLine = new VerticalLine(name);
        this->VerticalLines.push_back(newVLine);
        VerticalLinesId[name] = vLineSize;
        return newVLine;
    }

    return 0;
}

void vctPlot2DBase::Scale::ContinuousUpdate(void)
{
    if (this->Continuous) {
        if (this->ContinuousFitX && this->ContinuousFitY) {
            this->FitXY();
        } else if (this->ContinuousFitX) {
            this->FitX();
        } else if (this->ContinuousFitY) {
            this->FitY();
        }
    }
}


/***************************************************************************************************
    Trace Functions
***************************************************************************************************/

vctPlot2DBase::Trace::Trace(const std::string & name, size_t numberOfPoints, size_t PointSize):
    Name(name),
    Empty(true),
    Visible(true),
    Frozen(false),
    Buffer(0),
    PointSize(PointSize),
    IndexFirst(0),
    IndexLast(0),
    Color(1.0, 1.0, 1.0),
    LineWidth(1.0)
{
    // create the underlaying buffer and fill it with zeros
    CMN_ASSERT(PointSize >= 2);
    this->Buffer = new double[PointSize * numberOfPoints];
    memset(this->Buffer, 0, this->PointSize * numberOfPoints * sizeof(double));
    this->Data.SetSize(numberOfPoints);
    // now set all the references
    size_t index;
    for (index = 0;
         index < numberOfPoints;
         index++) {
        this->Data.Element(index).SetRef(this->Buffer + PointSize * index);
    }
}


vctPlot2DBase::Trace::~Trace()
{
    if (this->Buffer) {
        delete this->Buffer;
    }
}

vctPlot2DBase::Trace * vctPlot2DBase::Trace::AddTrace(const std::string & name){   
    return Parent->AddSignal(name);
}

vctPlot2DBase::Scale * vctPlot2DBase::Trace::GetParent(void){
    return Parent;
}

std::string vctPlot2DBase::Trace::GetName(void){
    return this->Name;
}

void vctPlot2DBase::Trace::AddPoint(const vctDouble2 & point)
{    
    if (!this->Frozen) {
        // look where to store this point
        if (!this->Empty) {
            this->IndexLast++;
            if (this->IndexLast == this->Data.size()) {
                this->IndexLast = 0;
            }
            if (this->IndexFirst == this->IndexLast ) {
                this->IndexFirst++;
                if (this->IndexFirst == this->Data.size()) {
                    this->IndexFirst = 0;
                }
            }
        } else {
            this->Empty = false;
        }
        this->Data.Element(IndexLast).Assign(point);
    }
}


void vctPlot2DBase::Trace::AppendPoint(const vctDouble2 & point)
{
    if (!this->Frozen) {
        // look where to store this point
        if (!this->Empty) {
            this->IndexLast++;
            if (this->IndexLast == this->Data.size()) {
                this->IndexLast = 0;
            }
            if (this->IndexFirst == this->IndexLast ) {
                this->IndexFirst++;
                if (this->IndexFirst == this->Data.size()) {
                    this->IndexFirst = 0;
                }
            }
        } else {
            this->Empty = false;
        }
        this->Data.Element(IndexLast).Assign(point);
    }
}


vctDouble2 vctPlot2DBase::Trace::GetPointAt(size_t index)
    throw (std::runtime_error)
{
    if (index >= Data.size()) {
        cmnThrow("vctPlot2DBase::Trace::GetPointAt: index bigger than bufferSize");
    }
    index = ((index + this->IndexFirst) % Data.size());
    return Data.at(index);
}


void vctPlot2DBase::Trace::SetPointAt(size_t index, const vctDouble2 & point)
    throw (std::runtime_error)
{
    if (index >= Data.size()) {
        cmnThrow("vctPlot2DBase::Trace::SetPointAt: index bigger than bufferSize");
    }
    index = ((index + this->IndexFirst) % Data.size());
    this->Data.Element(index).Assign(point);
    return;
}


void vctPlot2DBase::Trace::SetArrayAt(size_t index, const double * pointArray, size_t size, size_t pointDimension)
    throw (std::runtime_error)
{
    if (((index + (size/this->PointSize) ) >= Data.size())
        && (pointDimension != this->PointSize)) {
        cmnThrow("vctPlot2DBase::Trace::SetArrayAt: index + size bigger than bufferSize");
    }
    memcpy((this->Buffer + index * this->PointSize),
           pointArray,
           size * sizeof(double));
    return;
}


bool vctPlot2DBase::Trace::AppendArray(const double * pointArray, size_t arraySize, size_t pointDimension)
    throw (std::runtime_error)
{
    bool result = false;

    // check if there is enough space for array we want to insert
    if ((arraySize > (Data.size()*this->PointSize))
        || (pointDimension != this->PointSize)) {
        cmnThrow("vctPlot2DBase::Trace::AppendArray: arraySize invalid or pointDimension not match");
    }

    double * newBuffer = new double [Data.size() * this->PointSize];
    memset(newBuffer, 0, Data.size() * this->PointSize * sizeof(double));

    if (!this->Frozen) {
        size_t tempIndexLast = this->IndexLast, tempIndexFirst = this->IndexFirst;
        if (!this->Empty){
            if(tempIndexLast < tempIndexFirst ){
                // buffer full, move tempIndexFirst for pointArray
                tempIndexFirst += arraySize / this->PointSize;
                if (tempIndexFirst > Data.size()) {
                    tempIndexFirst = tempIndexFirst - Data.size() - 1;
                }
            } else {
                size_t sizeToMove = 0;
                sizeToMove = (Data.size() - this->IndexLast - 1);
                sizeToMove = (sizeToMove > (arraySize / this->PointSize)) ?
                    0 :
                    ((arraySize / this->PointSize) - sizeToMove);
                tempIndexFirst = tempIndexFirst + sizeToMove;
                if (tempIndexFirst == Data.size()) {
                    this->Empty = true;
                }
            }
        }
        size_t dataCopied = 0;
        if (!this->Empty) {
            if (tempIndexLast < tempIndexFirst){
                memcpy((newBuffer),
                       Buffer + (tempIndexFirst * this->PointSize),
                       (Data.size() - tempIndexFirst) * this->PointSize * sizeof(double));
                dataCopied = (Data.size() - tempIndexFirst);
                memcpy((newBuffer + (dataCopied * this->PointSize)),
                       Buffer,
                       (tempIndexLast + 1) * this->PointSize * sizeof(double));
                dataCopied += (tempIndexLast + 1);
            } else {
                memcpy((newBuffer),
                       Buffer + (tempIndexFirst * this->PointSize),
                       (tempIndexLast - tempIndexFirst +  1) * this->PointSize*sizeof(double));
                dataCopied = (tempIndexLast - tempIndexFirst + 1);
            }
        }
        memcpy((newBuffer + (dataCopied * this->PointSize)),
               pointArray,
               arraySize * sizeof(double));
        dataCopied += (arraySize / this->PointSize);
        delete Buffer;
        Buffer = newBuffer;
        for (size_t i = 0;
             i < Data.size();
             i++) {
            this->Data.Element(i).SetRef(this->Buffer + this->PointSize * i);
        }
        this->IndexFirst = 0;
        this->IndexLast = dataCopied - 1;
        this->Empty = false;
        result = true;
    }
    return result;
}



bool vctPlot2DBase::Trace::PrependArray(const double * pointArray, size_t arraySize, size_t pointDimension)
    throw (std::runtime_error)
{
    // check if there is enough space for array we want to insert
    bool result = false;

    if ((arraySize > (Data.size()*this->PointSize))
        || (pointDimension != this->PointSize)) {
        cmnThrow("vctPlot2DBase::Trace::PrependArray: arraySize invalid or pointDimension not match");
    }

    if (!this->Frozen) {
        int tempIndexLast = this->IndexLast, tempIndexFirst = this->IndexFirst;
        double * newBuffer = new double [Data.size()*this->PointSize];
        memset(newBuffer, 0, Data.size() * this->PointSize * sizeof(double));
        if (!this->Empty) {
            if (tempIndexLast < tempIndexFirst ) {
                // buffer full, move tempIndexFirst for pointArray
                tempIndexLast -= arraySize / this->PointSize;
                if(tempIndexLast < 0) {
                    tempIndexLast = Data.size() + tempIndexLast;
                }
            } else {
                size_t sizeToMove = 0;
                sizeToMove = (Data.size() - this->IndexLast - 1);
                sizeToMove = (sizeToMove > (arraySize / this->PointSize)) ?
                    0:
                    ((arraySize / this->PointSize) - sizeToMove);
                tempIndexLast = tempIndexLast - sizeToMove;
                if (tempIndexLast < 0) {
                    this->Empty = true;
                }
            }
        }
        size_t dataCopied = 0;
        if (!this->Empty) {
            if (tempIndexLast < tempIndexFirst) {
                memcpy((newBuffer + arraySize),
                       Buffer + (tempIndexFirst * this->PointSize),
                       (Data.size() - tempIndexFirst) * this->PointSize * sizeof(double));
                dataCopied = (Data.size() - tempIndexFirst);
                memcpy((newBuffer + arraySize + (dataCopied * this->PointSize)),
                       Buffer, (tempIndexLast + 1) * this->PointSize * sizeof(double));
                dataCopied += (tempIndexLast + 1);
            } else {
                memcpy((newBuffer + arraySize),
                       Buffer + (tempIndexFirst * this->PointSize),
                       (tempIndexLast - tempIndexFirst + 1) * this->PointSize*sizeof(double));
                dataCopied = (tempIndexLast - tempIndexFirst + 1);
            }
        }
        memcpy(newBuffer,
               pointArray,
               arraySize * sizeof(double));
        dataCopied += (arraySize / this->PointSize);
        delete Buffer;
        Buffer = newBuffer;
        for (size_t i = 0;
             i < Data.size();
             i++) {
            this->Data.Element(i).SetRef(this->Buffer + this->PointSize * i);
        }
        this->IndexFirst = 0;
        this->IndexLast = dataCopied - 1;
        this->Empty = false;
        result = true;
    }
    return result;
}


void vctPlot2DBase::Trace::Freeze(bool freeze)
{
    if (freeze != this->Frozen) {
        this->Frozen = freeze;
        if (!freeze) {
            this->IndexFirst = 0;
            this->IndexLast = 0;
            this->Empty = true;
        }
    }
}


void vctPlot2DBase::Trace::ComputeDataRangeXY(vctDouble2 & min, vctDouble2 & max) const
{
    // using pointer arithmetic
    const PointRef * currentPointer = this->Data.Pointer(0);
    size_t indexLast;
    const ptrdiff_t stridePointer = this->Data.stride();

    // if the buffer is full, just scan all elements otherwise scan up to last index
    if (this->IndexLast < this->IndexFirst) {
        indexLast = this->Data.size()-1;
    } else {
        indexLast = this->IndexLast;
    }
    const PointRef * lastPointer = currentPointer + indexLast * stridePointer;

    // iterate
    double value;
	max.X() = min.X() = currentPointer->X();
	max.Y() = min.Y() = currentPointer->Y();
    for (;
         currentPointer <= lastPointer;
         currentPointer += stridePointer) {
        value = currentPointer->X();
        if (value < min.X()) {
            min.X() = value;
        } else if (value > max.X()) {
            max.X() = value;
        }
        value = currentPointer->Y();
        if (value < min.Y()) {
            min.Y() = value;
        } else if (value > max.Y()) {
            max.Y() = value;
        }
    }

}


void vctPlot2DBase::Trace::GetLeftRightDataX(double & min, double & max) const
{
    min = Data.at(this->IndexFirst).X();
    max = Data.at(this->IndexLast).X();

}


void vctPlot2DBase::Trace::ComputeDataRangeX(double & min, double & max,  bool assumesDataSorted) const
{
    const PointRef * currentPointer = this->Data.Pointer(0);
    size_t indexLast;
    const ptrdiff_t stridePointer = this->Data.stride();

    if (assumesDataSorted) {
        min = Data.at(this->IndexFirst).X();
        max = Data.at(this->IndexLast).X();
        return;
    }

    // if the buffer is full, just scan all elements otherwise scan up to last index
    if (this->IndexLast < this->IndexFirst) {
        indexLast = this->Data.size()-1;
    } else {
        indexLast = this->IndexLast;
    }
    const PointRef * lastPointer = currentPointer + indexLast * stridePointer;

    // iterate
    double value;
	max = min = currentPointer->X();
    for (;
         currentPointer <= lastPointer;
         currentPointer += stridePointer) {
        value = currentPointer->X();
        if (value < min) {
            min = value;
        } else if (value > max) {
            max = value;
        }
    }
}


void vctPlot2DBase::Trace::ComputeDataRangeY(double & min, double & max) const
{
    // using pointer arithmetic
    const PointRef * currentPointer = this->Data.Pointer(0);
    size_t indexLast;
    const ptrdiff_t stridePointer = this->Data.stride();

    // if the buffer is full, just scan all elements otherwise scan up to last index
    if (this->IndexLast < this->IndexFirst) {
        indexLast = this->Data.size()-1;
    } else {
        indexLast = this->IndexLast;
    }
    const PointRef * lastPointer = currentPointer + indexLast * stridePointer;

    // iterate
    double value;
	min = max = currentPointer->Y();
    for (;
         currentPointer <= lastPointer;
         currentPointer += stridePointer) {
        value = currentPointer->Y();
        if (value < min) {
            min = value;
        } else if (value > max) {
            max = value;
        }
    }
}


void vctPlot2DBase::Trace::SetNumberOfPoints(size_t numberOfPoints)
{
    delete Buffer;
    Buffer = new double[PointSize * numberOfPoints];
    memset(this->Buffer, 0, this->PointSize * numberOfPoints * sizeof(double));
    IndexFirst = 0;
    IndexLast = 0;
    Empty = true;

    this->Data.SetSize(numberOfPoints);
    // now set all the references
    size_t index;
    for (index = 0;
         index < numberOfPoints;
         index++) {
        this->Data.Element(index).SetRef(this->Buffer + this->PointSize * index);
    }
}


size_t vctPlot2DBase::Trace::GetSize(void) const
{
    return Data.size();
}


size_t vctPlot2DBase::Trace::GetNumberOfPoints(void) const
{
    size_t numberOfPoints = 0;
    if (this->IndexFirst < this->IndexLast) {
        numberOfPoints = this->IndexLast - this->IndexFirst + 1;
    } else {
        numberOfPoints = Data.size() - this->IndexFirst + this->IndexLast + 1;
    }
    return numberOfPoints;
}


void vctPlot2DBase::Trace::GetNumberOfPoints(size_t & numberOfPoints, size_t & bufferSize) const
{
    if (this->IndexFirst < this->IndexLast) {
        numberOfPoints = this->IndexLast - this->IndexFirst + 1;
    } else {
        numberOfPoints = Data.size() - this->IndexFirst + this->IndexLast + 1;
    }
    bufferSize = Data.size();
    return ;
}


void vctPlot2DBase::Trace::SetSize(size_t numberOfPoints)
{
    delete Buffer;
    this->Buffer = new double[PointSize * numberOfPoints];
    memset(this->Buffer, 0, this->PointSize * numberOfPoints * sizeof(double));
    this->IndexFirst = 0;
    this->IndexLast = 0;
    Empty = true;

    this->Data.SetSize(numberOfPoints);
    // now set all the references
    size_t index;
    for (index = 0;
         index < numberOfPoints;
         index++) {
        this->Data.Element(index).SetRef(this->Buffer + this->PointSize * index);
    }
}


void vctPlot2DBase::Trace::Resize(size_t numberOfPoints, bool trimOlder)
{
    // same size, do nothing
    if (Data.size() == numberOfPoints) {
        return;
    }

    double * newBuffer;
    size_t dataAmount = (this->IndexLast - this->IndexFirst) + 1;
    if (dataAmount <= 0) {
        dataAmount = Data.size() + dataAmount;
    }

    // get a new one
    newBuffer = new double[this->PointSize * numberOfPoints];
    memset(newBuffer, 0, this->PointSize * numberOfPoints * sizeof(double));

    size_t tempIndexLast = 0;
    // we move index to where it should be, while shrinking
    if (numberOfPoints < dataAmount) {
        // we need to cut some data off
        if (trimOlder) {
            this->IndexFirst += (dataAmount - numberOfPoints);
            if (this->IndexFirst >= Data.size()) {
                // overflow, move to buffer head
                this->IndexFirst = this->IndexFirst - Data.size();
            }
        } else {
            this->IndexLast = this->IndexFirst + numberOfPoints - 1;
            if (this->IndexLast >= Data.size()) {
                this->IndexLast = this->IndexLast - Data.size();
            }
        }
        tempIndexLast = numberOfPoints- 1;
    } else {
        tempIndexLast = dataAmount - 1;
    }

    // data copy from IndexFirst to IndexLast
    if (this->IndexFirst <= this->IndexLast) {
        // continuous array
        size_t bytesToCopy = (this->IndexLast - this->IndexFirst + 1) * this->PointSize * sizeof(double);
        double * indexNow = this->Buffer + this->IndexFirst * this->PointSize;
        memcpy(newBuffer, indexNow, bytesToCopy);
    } else {
        // first part
        double * indexNow = (this->Buffer + this->PointSize * this->IndexFirst);
        size_t bytesToCopy = (Data.size() - this->IndexFirst) * this->PointSize * sizeof(double);
        memcpy(newBuffer, indexNow, bytesToCopy);
        // second part
        indexNow = this->Buffer;
        bytesToCopy = (this->IndexLast + 1)* this->PointSize * sizeof(double);
        memcpy((newBuffer + (Data.size() - this->IndexFirst) * this->PointSize),
               indexNow,
               bytesToCopy);
    }

    this->Data.SetSize(numberOfPoints);
    size_t index;
    for (index = 0;
         index < numberOfPoints;
         index++) {
        this->Data.Element(index).SetRef(newBuffer + this->PointSize * index);
    }
    delete Buffer;
    Buffer = newBuffer;
    this->IndexFirst = 0;
    this->IndexLast = tempIndexLast;
    return;
}


void vctPlot2DBase::Trace::SetColor(const vctDouble3 & colorInRange0To1)
{
    vctDouble3 clippedColor;
    clippedColor.ClippedAboveOf(colorInRange0To1, 1.0);
    clippedColor.ClippedBelowOf(clippedColor, 0.0);   
    this->Color.Assign(clippedColor);
}

/***************************************************************************************************
    VerticalLine Functions
***************************************************************************************************/

vctPlot2DBase::VerticalLine::VerticalLine(const std::string & name, const double x):
    Name(name),
    X(x),
    Visible(true),
    Color(1.0, 1.0, 1.0),
    LineWidth(1.0)
{
}


vctPlot2DBase::VerticalLine::~VerticalLine()
{
};


void vctPlot2DBase::VerticalLine::SetX(const double x)
{
    this->X = x;
}


void vctPlot2DBase::VerticalLine::SetColor(const vctDouble3 & colorInRange0To1)
{
    vctDouble3 clippedColor;
    clippedColor.ClippedAboveOf(colorInRange0To1, 1.0);
    clippedColor.ClippedBelowOf(clippedColor, 0.0);
    this->Color.Assign(clippedColor);
}


/***************************************************************************************************
    Plot Funcitons
***************************************************************************************************/

vctPlot2DBase::vctPlot2DBase(size_t pointSize):
    PointSize(pointSize),
    Frozen(false),
    BackgroundColor(0.1, 0.1, 0.1)
{
    CMN_ASSERT(this->PointSize >= 2);
    SetNumberOfPoints(200);
    Translation.SetAll(0.0);
    ScaleValue.SetAll(1.0);
    SetContinuousFitX(true);
    SetContinuousFitY(true);
}


// TODO: This must be modified into AddScale + AddSignal

vctPlot2DBase::Trace * vctPlot2DBase::AddTrace(const std::string & name, size_t & traceId)
{
    // check if the name already exists
    const std::string delimiter("-");
    std::string scaleName;
    size_t delimiterPosition = name.find(delimiter);
    Scale * scalePointer = this->FindScale(name);
    
    scaleName = name.substr(0, delimiterPosition);


    // no such Scale, create one
    if(!scalePointer){
        std::cerr<<"AddScale now\n";
        scalePointer = this->AddScale(scaleName);
    }

    return scalePointer->AddSignal(name);
}

bool vctPlot2DBase::RemoveTrace(const std::string & name){
    const std::string delimiter("-");
    std::string scaleName;
    size_t delimiterPosition = name.find(delimiter);
    Scale * scalePointer = this->FindScale(name);
    
    scaleName = name.substr(0, delimiterPosition);
    
    if(!scalePointer) // not found
        return false;

    return scalePointer->RemoveSignal(name);
}


// vctPlot2DBase::Trace * vctPlot2DBase::AddTrace(const std::string & name, size_t & traceId)
// {
//     // check if the name already exists
//     TracesIdType::const_iterator found = this->TracesId.find(name);
//     const TracesIdType::const_iterator end = this->TracesId.end();
//     if (found == end) {
//         // new name
//         traceId = Traces.size();
//         Trace * newTrace = new Trace(name, this->NumberOfPoints, this->PointSize);
//         Traces.push_back(newTrace);
//         TracesId[name] = traceId;
//         return newTrace;
//     }
//     return 0;
// }


vctPlot2DBase::Trace * vctPlot2DBase::AddTrace(const std::string & name)
{
    size_t traceId;   
    return this->AddTrace(name, traceId);
}

vctPlot2DBase::VerticalLine * vctPlot2DBase::AddVerticalLine(const std::string & name)
{
    TracesIdType::const_iterator found;
    const TracesIdType::const_iterator end = this->ScalesId.end();
    const std::string delimiter("-");
    std::string scaleName;
    size_t delimiterPosition = name.find(delimiter);
    Scale * scalePointer = this->FindScale(name);
    
    scaleName = name.substr(0, delimiterPosition);


    // no such Scale, create one
    if(!scalePointer){
        std::cerr<<"AddScale now\n";
        scalePointer = this->AddScale(scaleName);
    }

    return scalePointer->AddVerticalLine(name);

}

/*
vctPlot2DBase::VerticalLine * vctPlot2DBase::AddVerticalLine(const std::string & name)
{
    VerticalLine * newLine = new VerticalLine(name);
    if (this->VerticalLines.AddItem(name, newLine)) {
        return newLine;
    }
    delete newLine;
    return 0;
}
*/

void vctPlot2DBase::SetNumberOfPoints(size_t numberOfPoints)
{
    this->NumberOfPoints = numberOfPoints;
    if (this->NumberOfPoints < 3) {
        this->NumberOfPoints = 3;
    }
}


void vctPlot2DBase::AddPoint(size_t traceId, const vctDouble2 & point)
{
    this->Traces[traceId]->AppendPoint(point);
}


void vctPlot2DBase::FitX(double padding)
{
    double min, max;
    this->ComputeDataRangeX(min, max);
    this->FitX(min, max, padding);
}


void vctPlot2DBase::FitX(double min, double max, double padding)
{    
    this->ViewingRangeX.Assign(min, max);
    this->ScaleValue.X() = this->Viewport.X() / ((max - min) * (1.0 + padding));
    this->Translation.X() =
        - min * this->ScaleValue.X()
        + 0.5 * padding * this->Viewport.X();

    size_t scaleIndex;
    const size_t numberofScales = this->Scales.size();

    for(scaleIndex = 0; scaleIndex < numberofScales; scaleIndex++)
        this->Scales[scaleIndex]->FitX(min,max, padding);    
}


void vctPlot2DBase::FitY(double padding)
{
    double min, max;
    this->ComputeDataRangeY(min, max);
    this->FitY(min, max, padding);
}


void vctPlot2DBase::FitY(double min, double max, double padding)
{

    this->ViewingRangeY.Assign(min, max);
    this->ScaleValue.Y() = this->Viewport.Y() / ((max - min) * (1.0 + padding));
    this->Translation.Y() =
        - min * this->ScaleValue.Y()
        + 0.5 * padding * this->Viewport.Y();

    size_t scaleIndex;
    const size_t numberofScales = this->Scales.size();

    for(scaleIndex = 0; scaleIndex < numberofScales; scaleIndex++)
        this->Scales[scaleIndex]->FitY(min, max, padding);
}


void vctPlot2DBase::FitXY(const vctDouble2 & padding)
{
    vctDouble2 min, max;
    this->ComputeDataRangeXY(min, max);
    this->FitXY(min, max, padding);
}


void vctPlot2DBase::FitXY(vctDouble2 min, vctDouble2 max, const vctDouble2 & padding)
{
    size_t scaleIndex;
    const size_t numberofScales = this->Scales.size();

    for(scaleIndex = 0; scaleIndex < numberofScales; scaleIndex++)
        this->Scales[scaleIndex]->FitXY(min, max, padding);    
}


void vctPlot2DBase::Freeze(bool freeze)
{
    size_t traceIndex;
    const size_t numberOfTraces = this->Traces.size();
    for (traceIndex = 0;
         traceIndex < numberOfTraces;
         traceIndex++) {
        this->Traces[traceIndex]->Freeze(freeze);
    }
    this->Frozen = freeze;
}


void vctPlot2DBase::SetContinuousFitX(bool fit)
{
    this->ContinuousFitX = fit;
    this->ContinuousAlignMaxX = false;
    this->Continuous = (this->ContinuousFitX
                        || this->ContinuousFitY
                        || this->ContinuousAlignMaxX);
    size_t scaleIndex;
    const size_t numberofScales = this->Scales.size();

    for(scaleIndex = 0; scaleIndex < numberofScales; scaleIndex++)
        this->Scales[scaleIndex]->SetContinuousFitX(fit);
}


void vctPlot2DBase::SetContinuousFitY(bool fit)
{
    this->ContinuousFitY = fit;
    this->Continuous = (this->ContinuousFitX
                        || this->ContinuousFitY
                        || this->ContinuousAlignMaxX);
    size_t scaleIndex;
    const size_t numberofScales = this->Scales.size();

    for(scaleIndex = 0; scaleIndex < numberofScales; scaleIndex++)
        this->Scales[scaleIndex]->SetContinuousFitY(fit);

}


void vctPlot2DBase::SetContinuousAlignMaxX(bool align)
{
    this->ContinuousAlignMaxX = align;
    this->ContinuousFitX = false;
    this->Continuous = (this->ContinuousFitX
                        || this->ContinuousFitY
                        || this->ContinuousAlignMaxX);
}


void vctPlot2DBase::ComputeDataRangeX(double & min, double & max)
{
    if(this->Scales.size() == 0){
        min = -1.0;
        max = 1.0;
    } else {
        size_t scaleIndex;
        const size_t numberofScales = this->Scales.size();

        for(scaleIndex = 0;
            scaleIndex < numberofScales;
            scaleIndex++)
            this->Scales[scaleIndex]->ComputeDataRangeX(min, max);        
    }
}


void vctPlot2DBase::ComputeDataRangeY(double & min, double & max)
{
    if(this->Scales.size() == 0){
        min = -1.0;
        max = 1.0;
    } else {
        size_t scaleIndex;
        const size_t numberofScales = this->Scales.size();

        for(scaleIndex = 0; scaleIndex < numberofScales; scaleIndex++)
            this->Scales[scaleIndex]->ComputeDataRangeY(min, max);
    }
}


void vctPlot2DBase::ComputeDataRangeXY(vctDouble2 & min, vctDouble2 & max)
{
    if (this->Traces.size() == 0) {
        min.SetAll(-1.0);
        max.SetAll(1.0);
    } else {

        size_t scaleIndex;
        const size_t numberofScales = this->Scales.size();

        for(scaleIndex = 0; scaleIndex < numberofScales; scaleIndex++)
            this->Scales[scaleIndex]->ComputeDataRangeXY(min, max);
    }
}


void vctPlot2DBase::ContinuousUpdate(void)
{
    // call each Scale->ContinuousUpdate

    size_t scaleIndex;
    const size_t numberofScales = this->Scales.size();

    for(scaleIndex = 0; scaleIndex < numberofScales; scaleIndex++)
        this->Scales[scaleIndex]->ContinuousUpdate();

}


// void vctPlot2DBase::ContinuousUpdate(void)
// {
//     if (this->Continuous) {
//         if (this->ContinuousFitX && this->ContinuousFitY) {
//             this->FitXY();
//             std::cerr<<"1\n";
//         } else if (this->ContinuousFitX) {
//             this->FitX();
//             std::cerr<<"2\n";
//         } else if (this->ContinuousFitY) {
//             this->FitY();
//             std::cerr<<"3\n";
//         }
//     }
// }


void vctPlot2DBase::SetColor(size_t traceId, const vctDouble3 & colorInRange0To1)
{
    this->Traces[traceId]->SetColor(colorInRange0To1);
}


void vctPlot2DBase::SetBackgroundColor(const vctDouble3 & colorInRange0To1)
{
    vctDouble3 clippedColor;
    clippedColor.ClippedAboveOf(colorInRange0To1, 1.0);
    clippedColor.ClippedBelowOf(clippedColor, 0.0);
    this->BackgroundColor.Assign(clippedColor);
}

vctPlot2DBase::Scale * vctPlot2DBase::AddScale(const std::string & name){
    // check if the name already exists
    TracesIdType::const_iterator found = this->ScalesId.find(name);
    const TracesIdType::const_iterator end = this->ScalesId.end();
    if (found == end) {
        size_t ScaleSize = this->Scales.size();
        // new name
        //set the number of points to default, 100       
        Scale * newScale = new Scale(name);
        this->Scales.push_back(newScale);
        ScalesId[name] = ScaleSize;
        // temporary solution
        //vctPlot2DBase::Traces = Scales[0]->Traces;
        return newScale;
    }    
    return 0;    
}

vctPlot2DBase::Scale * vctPlot2DBase::FindScale(const std::string & name){
    // check if the name already exists   
    TracesIdType::const_iterator found;
    const TracesIdType::const_iterator end = this->ScalesId.end();
    const std::string delimiter("-");
    std::string scaleName;
    size_t delimiterPosition = name.find(delimiter);
    
    scaleName = name.substr(0, delimiterPosition);

    found = this->ScalesId.find(scaleName);    
    
    // no match
    if (found == end) 
        return 0;
    else
        return Scales[found->second];
}
