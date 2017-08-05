//
//  Created by Al Bernstein 2017/07/06
//  Copyright 2017 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "HighVelocityFilter.h"



namespace controller {

    HighVelocityFilter::HighVelocityFilter() {
        
        _pThresh = 0.0;
        _pWeight = 0.0f;
        _qWeight = 0.0f;
        _posRingIndex = 0;
        _rotRingIndex = 0;
        _magRingIndex = 0;
        _ringBack = (_ringSize - 1) / 2;
        _numberSamples = 0;
    }

    HighVelocityFilter::HighVelocityFilter(float pThreshold, float pWeight, float qThreshold, float qWeight, uint size){

        _pThresh = pThreshold;
        _pWeight = pWeight;
        _qWeight = qWeight;
        _posRingIndex = 0;
        _rotRingIndex = 0;
        _magRingIndex = 0;
        _ringSize = size;
        _ringBack = (_ringSize - 1) / 2;
        _numberSamples = 0; 

  
    }

    HighVelocityFilter::HighVelocityFilter(const HighVelocityFilter &other) {
       
        _pThresh = other._pThresh;
        _pWeight = other._pWeight;
        _qWeight = other._qWeight;
        _posRingIndex = other._posRingIndex;
        _rotRingIndex = other._rotRingIndex;
        _magRingIndex = other._magRingIndex;
        _ringSize = other._ringSize;
        _ringBack = other._ringBack;
        _numberSamples = other._numberSamples;
    }

    HighVelocityFilter::~HighVelocityFilter() {
        _magRingBuffer.clear();
        _posRingBuffer.clear();
        _rotRingBuffer.clear();
    }


    glm::vec3 HighVelocityFilter::ringBufferManager(glm::vec3 v, uintptr_t size) const {
    
        uintptr_t len = getPosRingBufferSize();
        glm::vec3 ret = v;
        uintptr_t index = 0;

        if (len < size) {
            setPosRingBuffer(v);
        }
        else {
            uintptr_t posIndex = getPosRingIndex();
            uintptr_t ringBack = getRingBack();
            index = ( 1 + posIndex + ringBack) % size;
            ret = getPosRingBuffer(index);
            setPosRingBuffer(v, posIndex);
            posIndex++;
            posIndex = posIndex % size;
            setPosRingIndex(posIndex);
        }

       
        #if WANT_DEBUG

        uintptr_t len1 = _magRingBuffer.size();

        qDebug() << " Pos Buffer: _posRingIndex = " << _posRingIndex << " index = " << index << " _ringBack = " << _ringBack << " size = " << size << " length before = " << len << " length after = " << len1 << endl;
        for (uintptr_t i = 0; i < len1; i++) {
            qDebug() << i << "\t" << _posRingBuffer[i].x << "\t" << _posRingBuffer[i].y << "\t" << _posRingBuffer[i].z << endl;
        }

        #endif

        return ret;
}
       

    glm::quat HighVelocityFilter::ringBufferManager(glm::quat q, uintptr_t size) const {

        uintptr_t len = getRotRingBufferSize();
        glm::quat ret = q;
        uintptr_t index = 0;

        if (len < size) {
            setRotRingBuffer(q);
        }
        else {
            uintptr_t rotRingIndex = getRotRingIndex();
            uintptr_t ringBack = getRingBack();
            index = ( 1 + rotRingIndex + ringBack) % size;

            ret = getRotRingBuffer(index);
            setRotRingBuffer(q, rotRingIndex);
            rotRingIndex++;
            rotRingIndex = rotRingIndex % size;
            setRotRingIndex(rotRingIndex);
        }

       #if WANT_DEBUG
        // write out buffer
        uintptr_t len1 = _rotRingBuffer.size();
        qDebug() << " Rot Buffer: _rotRingIndex = " << _rotRingIndex << " index =  " << index << " _ringBack = " << _ringBack << " size = " << size << " length before = " << len << " length after = " << len1 << endl;
        for (uintptr_t i = 0; i < len1; i++) {
            qDebug() << i << "\t" << _rotRingBuffer[i].w << "\t" << _rotRingBuffer[i].x << "\t" << _rotRingBuffer[i].y << "\t" << _rotRingBuffer[i].z << endl;
        }
         #endif


        return ret;
    }


    float HighVelocityFilter::ringBufferManager(float mag, uintptr_t size) const {


        uintptr_t len = getMagRingBufferSize();
        float ret = mag;
        uint index = 0;

        if (len < size) {
            setMagRingBuffer(mag);
        }
        else {
            uintptr_t magRingIndex = getMagRingIndex();
            uintptr_t ringBack = getRingBack();

            index = (1 + magRingIndex + ringBack) % size;
            ret = getMagRingBuffer(index);
            setMagRingBuffer(magRingIndex,mag);
            magRingIndex++;
            magRingIndex = magRingIndex % size;
            setMagRingIndex(magRingIndex);
        }

       
        #if WANT_DEBUG
        // write out buffer
            uintptr_t len1 = _magRingBuffer.size();
            qDebug() << " Mag Buffer: _magRingIndex = " << _magRingIndex << " index = " << index << " _ringBack = " << _ringBack << " size = " << size
                     << " length before = " << len << " length after = " << len1 << endl;
            for (uintptr_t i = 0; i < len1; i++) {
                qDebug() << i << "\t" << _magRingBuffer[i] << endl;
            }
        #endif

        return ret;
    }


    Pose HighVelocityFilter::apply(Pose newPose) const {

    
        Pose ret;

        glm::vec3 pos = newPose.getTranslation();
        glm::quat rot = newPose.getRotation();
        glm::vec3 vel = newPose.getVelocity();
        glm::vec3 a_vel = newPose.getAngularVelocity();
        bool valid = newPose.isValid();

        ret.translation = pos;
        ret.rotation = rot;
        ret.velocity = vel;
        ret.angularVelocity = a_vel;
        ret.valid = valid;

       setNotZeroFlag(glm::dot(pos, pos) != 0.0f); 

        #if WANT_DEBUG
        if (glm::dot(_notZeroFlag) != 0.0f) {
            qDebug() << " Filter Input: " << " " << pos.x << " " << pos.y << " " << pos.z << " "
                << rot.w << " " << rot.x << " " << rot.y << " " << rot.z;
               // << "velocity: " << vel.x << " " << vel.y << " " << vel.z << " "
               // << "angular velocity: " << " " << a_vel.x << " " << a_vel.y << " " << a_vel.z << " " 
               // << "valid: " <<valid;
        }
        #endif

        uintptr_t posRingIndex = getPosRingIndex();
        uintptr_t ringBack = getRingBack();
        uintptr_t ringSize = getRingSize();
        uintptr_t magRingIndex = getMagRingIndex();

        if (getNotZeroFlag()) {

            uintptr_t numSamples = getNumberSamples();
            float pWeight = getPosWeight();


            if ( numSamples == 0) {
                setPosAverage( { 0.0f, 0.0f, 0.0f } );
            }
            else {
                glm::vec3 posAvg = getPosAverage();
                posAvg = pos * pWeight + (1.0f - pWeight)*posAvg;
                setPosAverage(posAvg);
            }

            glm::vec3 posAvg = getPosAverage();

            glm::vec3 dv = pos - posAvg;
            float dvMag = sqrtf(glm::dot(dv, dv));
            float signal = ringBufferManager(dvMag, ringSize);
            glm::vec3 vTmp = ringBufferManager(pos, ringSize);
            glm::quat qTmp = ringBufferManager(rot, ringSize);

            numSamples++;
            setNumSamples(numSamples);

            #if WANT_DEBUG
            if (_notZeroFlag) {
                qDebug() << " After Ring Buffer - Input: " << " " << vTmp.x << " " << vTmp.y << " " << vTmp.z << " "
                    << qTmp.w << " " << qTmp.x << " " << qTmp.y << " " << qTmp.z;
                     // " " << signal << endl;
               //qDebug() << "threshold:\t " << _pThresh << "_numberSamples:\t " << _numberSamples << "_pSamples:\t " << _pSamples <<endl;
            }
             #endif


            if ( numSamples >= getAvgLength() ){
                uint index = 0;

                if (signal >= getPosThreshold() ) {

                    glm::vec3 begin = getPosRingBuffer(posRingIndex); // first
                    index = (posRingIndex - 1) % ringSize;
                    glm::vec3 end = getPosRingBuffer(index); // last
                    
                    // write out buffer

                   // qDebug() << " Load Position Buffer " << endl;
                     
                    #if WANT_DEBUG
                    // write out buffer before
                    qDebug() << " Pos Buffer Before: _ringIndex = " << getPosRingIndex() << endl;
                    uintptr_t len = getPosRingBufferSize();
                    for (uintptr_t i = 0; i < len; i++) {
                        glm::vec3 pos = getPosRingBuffer(i);
                        qDebug() << i << "\t" << pos.x << "\t" << pos.y << "\t" << pos.z << endl;
                    }
                    #endif

                    // set up step in pos ring buffer

                    //#if FILTER_OFF
                    
                    setPosRingBuffer(begin, posRingIndex);

                    uint start = (posRingIndex + 1) % ringSize;

                    for (uint i = start; i < start + ringSize - 1; i++) {
                        index = i % ringSize;
                        #if WANT_DEBUG
                        qDebug() << "index = " << index << endl;
                        #endif
                        setPosRingBuffer(end, index);
                    }
                   

                    // average over position step function

                    posAvg = begin;

                    for ( uint i = start; i < start + ringSize - 1; i++) {
                        index = i % ringSize;
                        #if WANT_DEBUG
                        qDebug() << "index = " << index << endl;
                        #endif
                        pos = getPosRingBuffer(index);
                        posAvg = pos * pWeight + (1.0f - pWeight)*posAvg;
                        setPosRingBuffer(posAvg, index);
                    }

                   #if WANT_DEBUG
                    // write out buffer after
                    qDebug() << " Pos Buffer After: _ringIndex = " << _posRingIndex;
                    len = _posRingBuffer.size();
                    for (uint i = 0; i < len; i++) {
                        qDebug() << i << "\t" << _posRingBuffer[i].x << "\t" << _posRingBuffer[i].y << "\t" << _posRingBuffer[i].z;
                    }

                    #endif


                    // set up step in rotation ring buffer

                    uintptr_t rotRingIndex = getRotRingIndex();
                 
                    glm::quat qBegin = getRotRingBuffer(rotRingIndex);  // first
                    index = (rotRingIndex - 1) % ringSize;
                    glm::quat qEnd = getRotRingBuffer(index); // last
                    
                    setRotRingBuffer(qBegin, rotRingIndex);
                    uintptr_t qstart = (rotRingIndex + 1) % ringSize;


                    for (uintptr_t i = qstart; i < qstart + ringSize - 1; i++) {
                        index = i % ringSize;
                        #if WANT_DEBUG
                        qDebug() << "index = " << index << endl;
                        #endif
                        setRotRingBuffer(qEnd, index);
                    }


                    // average over rotation step function

                    float qWeight = getRotWeight();

                    glm::quat qAvg = qBegin;

                    for (uintptr_t i = start; i < start + ringSize - 1; i++) {
                        index = i % ringSize;
                        #if WANT_DEBUG
                        qDebug() << "index = " << index << endl;
                        #endif
                        glm::quat rot = getRotRingBuffer(index);
                        qAvg = rot * qWeight + (1.0f - qWeight)*qAvg;
                        setRotRingBuffer(qAvg, index);
                    }

                    // clear signal buffer

                    for (uintptr_t i = 0; i < ringSize; i++){
                        index = i % ringSize;
                        setMagRingBuffer(0.0f, index);
                    }
                      
                    // copy position ring buffer to output

                    for (uintptr_t i = posRingIndex; i < posRingIndex + ringSize; i++) {
                        uintptr_t index = i % ringSize;
                        glm::vec3 vTmp = getPosRingBuffer(index);
                        std::vector<glm::vec3>::iterator it = getPosBufferBegin();
                        setPosBuffer(it, vTmp);
                    }
                        
                    // copy rotation ring buffer to the output 

                    for (uintptr_t i = rotRingIndex; i < rotRingIndex + ringSize; i++) {
                        uintptr_t index = i % ringSize;
                        glm::quat qTmp = getRotRingBuffer(index);
                        std::vector<glm::quat>::iterator it = getRotBufferBegin();
                        setRotBuffer(it, qTmp);
                    }

                    //#endif

                     #if WANT_DEBUG
                    // write out buffer after
                    qDebug() << " Pos Buffer After: _ringIndex = " << _posRingIndex << endl;
                    len = _posRingBuffer.size();
                    for (uintptr_t i = 0; i < len; i++) {
                        qDebug() << i << "\t" << _posRingBuffer[i].x << "\t" << _posRingBuffer[i].y << "\t" << _posRingBuffer[i].z;
                    }
                    

                    // write out buffer after
                    qDebug() << " Mag Buffer After: _ringIndex = " << _magRingIndex << endl;

                    len = _magRingBuffer.size();
                    for (uintptr_t i = 0; i < len; i++) {
                        qDebug() << i << "\t" << _magRingBuffer[i] << endl;
                    }
                     #endif
                }
                else {
                    std::vector<glm::vec3>::iterator it = getPosBufferBegin();
                    setPosBuffer(it, vTmp);
                
                    std::vector<glm::quat>::iterator it1 = getRotBufferBegin();
                    setRotBuffer(it1, qTmp);
                }

                index = (posRingIndex + ringBack) % ringSize;
                glm::vec3 vTmp = getPosRingBuffer(index);
                glm::quat qTmp = getRotRingBuffer(index);
                index = (magRingIndex + ringBack) % ringSize;
                signal = getMagRingBuffer(index);
                ret.translation = vTmp;
                ret.rotation = qTmp;

                #if WANT_DEBUG
                qDebug() << " Output: " << vTmp.x << "\t" << vTmp.y << "\t" << vTmp.z << "\t"
                         << "\t" << "Rotation: " << rot.w << "\t "<< rot.x << "\t " << rot.y << "\t " << rot.z;
                #endif
            }

        }

        uintptr_t len = getPosBufferSize();
        if (len > 0) {
            ret.translation = getPosBuffer(len - 1);
            posBufferPop();
        }
        else {
            ret.translation = pos;
        }
       
        #if WANT_DEBUG
        qDebug() << " Pos Output Buffer _posBuffer,  length = " << len << endl;
        for (uintptr_t i = 0; i < len; i++) {
            qDebug() << i << "\t" << _posBuffer[i].x << "\t" << _posBuffer[i].y << "\t" << _posBuffer[i].z;
        }
        #endif
         
        len = getRotationBufferSize();
        if (len > 0) {
            ret.rotation = getRotBuffer(len-1);
            rotBufferPop();
        }
        else {
            ret.rotation = rot;
        }

        #if WANT_DEBUG

        if (_notZeroFlag) {
            qDebug() << " Filter Output: " << vTmp.x << " " << vTmp.y << " " << vTmp.z << " " << qTmp.w << " " << qTmp.x << " " << qTmp.y << " " << qTmp.z;
        }

        #endif

        return ret;
    }


bool HighVelocityFilter::parseParameters(const QJsonValue& parameters) {

        if (parameters.isObject()) {
            auto obj = parameters.toObject();
            if (obj.contains(JSON_P_THRESHOLD) && obj.contains(JSON_SIZE) && 
                obj.contains(JSON_P_WEIGHT) && obj.contains(JSON_Q_WEIGHT) && obj.contains(JSON_P_SAMPLES)) {
                _pThresh = obj[JSON_P_THRESHOLD].toDouble();
                _pWeight = obj[JSON_P_WEIGHT].toDouble();
                _ringSize = obj[JSON_SIZE].toInt();
                _qWeight = obj[JSON_Q_WEIGHT].toDouble();
                _avgLength = obj[JSON_P_SAMPLES].toInt();
                return true;
            }
        }
        return false;
    }
}