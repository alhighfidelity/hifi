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


    glm::vec3 HighVelocityFilter::ringBufferManager(glm::vec3 v, uint size) const {
    
        uintptr_t len = _posRingBuffer.size();
        glm::vec3 ret = v;
        uintptr_t index = 0;

        if (len < size) {
            _posRingBuffer.push_back(v);
        }
        else {
            index = ( 1 + _posRingIndex + _ringBack) % size;
            ret = _posRingBuffer[index];
            _posRingBuffer[_posRingIndex] = v;
            _posRingIndex++;
            _posRingIndex = _posRingIndex % size;
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
       

    glm::quat HighVelocityFilter::ringBufferManager(glm::quat q, uint size) const {

        uintptr_t len = _rotRingBuffer.size();
        glm::quat ret = q;
        uintptr_t index = 0;

        if (len < size) {
            _rotRingBuffer.push_back(q);
        }
        else {
            index = ( 1 + _rotRingIndex + _ringBack) % size;
            ret = _rotRingBuffer[index];
            _rotRingBuffer[_rotRingIndex] = q;
            _rotRingIndex++;
            _rotRingIndex = _rotRingIndex % size;
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

        uintptr_t len = _magRingBuffer.size();
        float ret = mag;
        uint index = 0;

        if (len < size) {
            _magRingBuffer.push_back(mag);
        }
        else {
            index = (1 + _magRingIndex + _ringBack) % size;
            ret = _magRingBuffer[index];
            _magRingBuffer[_magRingIndex] = mag;
            _magRingIndex++;
            _magRingIndex = _magRingIndex % size;
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

       _notZeroFlag = glm::dot(newPose.getTranslation(), newPose.getTranslation()) != 0.0f; 

        #if WANT_DEBUG
        if (glm::dot(pos, pos) != 0.0f) {
            qDebug() << " Filter Input: " << " " << pos.x << " " << pos.y << " " << pos.z << " "
                << rot.w << " " << rot.x << " " << rot.y << " " << rot.z;
               // << "velocity: " << vel.x << " " << vel.y << " " << vel.z << " "
               // << "angular velocity: " << " " << a_vel.x << " " << a_vel.y << " " << a_vel.z << " " 
               // << "valid: " <<valid;
        }
        #endif


        if (_notZeroFlag) {

            if (_numberSamples == 0) {
                _posAvg = { 0.0f, 0.0f, 0.0f };
            }
            else {
                _posAvg = pos * _pWeight + (1.0f - _pWeight)*_posAvg;
            }

            glm::vec3 avg = _posAvg;

            glm::vec3 dv = pos - avg;
            float dvMag = sqrtf(glm::dot(dv, dv));
            float signal = ringBufferManager(dvMag, _ringSize);
            glm::vec3 vTmp = ringBufferManager(pos, _ringSize);
            glm::quat qTmp = ringBufferManager(rot, _ringSize);

            _numberSamples++;

            #if WANT_DEBUG
            if (glm::dot(vTmp, vTmp) != 0.0f) {
                qDebug() << " After Ring Buffer - Input: " << " " << vTmp.x << " " << vTmp.y << " " << vTmp.z << " "
                    << qTmp.w << " " << qTmp.x << " " << qTmp.y << " " << qTmp.z;
                     // " " << signal << endl;
               //qDebug() << "threshold:\t " << _pThresh << "_numberSamples:\t " << _numberSamples << "_pSamples:\t " << _pSamples <<endl;
            }
             #endif



            if (_numberSamples >= _avgLength){
                uint index = 0;

                if (signal >= _pThresh) {

                    index = _posRingIndex;
                    glm::vec3 begin = _posRingBuffer[index];  // first
                    index = (_posRingIndex - 1) % _ringSize;
                    glm::vec3 end = _posRingBuffer[index]; // last

                    // write out buffer

                   // qDebug() << " Load Position Buffer " << endl;
                     
                    #if WANT_DEBUG
                    // write out buffer before
                    qDebug() << " Pos Buffer Before: _ringIndex = " << _posRingIndex << endl;
                    uintptr_t len = _posRingBuffer.size();
                    for (uintptr_t i = 0; i < len; i++) {
                        qDebug() << i << "\t" << _posRingBuffer[i].x << "\t" << _posRingBuffer[i].y << "\t" << _posRingBuffer[i].z << endl;
                    }
                    #endif

                    // set up step in pos ring buffer

                    //#if FILTER_OFF

                    _posRingBuffer[_posRingIndex] = begin;
                    uint start = (_posRingIndex + 1) % _ringSize;

                    for (uint i = start; i < start + _ringSize - 1; i++) {
                        index = i%_ringSize;
                        #if WANT_DEBUG
                        qDebug() << "index = " << index << endl;
                        #endif
                        _posRingBuffer[index] = end;
                    }
                   

                    // average over position step function

                    avg = begin;


                    for ( uint i = start; i < start + _ringSize - 1; i++) {
                        index = i%_ringSize;
                        #if WANT_DEBUG
                        qDebug() << "index = " << index << endl;
                        #endif
                        avg = _posRingBuffer[index] * _pWeight + (1.0f - _pWeight)*avg;
                        _posRingBuffer[index] = avg;
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

                    index = _rotRingIndex;
                    glm::quat qBegin = _rotRingBuffer[index];  // first
                    index = (_rotRingIndex - 1) % _ringSize;
                    glm::quat qEnd = _rotRingBuffer[index]; // last

                    _rotRingBuffer[_rotRingIndex] = qBegin;
                    uintptr_t qstart = (_rotRingIndex + 1) % _ringSize;


                    for (uintptr_t i = qstart; i < qstart + _ringSize - 1; i++) {
                        index = i%_ringSize;
                        #if WANT_DEBUG
                        qDebug() << "index = " << index << endl;
                        #endif
                        _rotRingBuffer[index] = qEnd;
                    }


                    // average over rotation step function

                    glm::quat qAvg = qBegin;

                    for (uintptr_t i = start; i < start + _ringSize - 1; i++) {
                        index = i%_ringSize;
                        #if WANT_DEBUG
                        qDebug() << "index = " << index << endl;
                        #endif
                        qAvg = _rotRingBuffer[index] * _qWeight + (1.0f - _qWeight)*qAvg;
                        _rotRingBuffer[index] = qAvg;
                    }

                    

                    // clear signal buffer

                    for (uintptr_t i = 0; i < _ringSize; i++){
                        index = i%_ringSize;
                        _magRingBuffer[index] = 0.0f;
                    }
                      

                    // copy position ring buffer to output


                    for (uintptr_t i = _posRingIndex; i < _posRingIndex + _ringSize; i++) {
                        uintptr_t index = i%_ringSize;
                        glm::vec3 vTmp = _posRingBuffer[index];
                        std::vector<glm::vec3>::iterator it = _posBuffer.begin();
                        _posBuffer.insert(it, vTmp);
                    }
                        
                    // copy rotation ring buffer to the output 


                    for (uintptr_t i = _rotRingIndex; i < _rotRingIndex + _ringSize; i++) {
                        uintptr_t index = i%_ringSize;
                        glm::quat qTmp = _rotRingBuffer[index];
                        std::vector<glm::quat>::iterator it = _rotBuffer.begin();
                        _rotBuffer.insert(it, qTmp);
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
                    std::vector<glm::vec3>::iterator it = _posBuffer.begin();
                    _posBuffer.insert(it, vTmp);
                    std::vector<glm::quat>::iterator it1 = _rotBuffer.begin();
                    _rotBuffer.insert(it1,qTmp);
                }

                index = (_posRingIndex + _ringBack) % _ringSize;
                glm::vec3 vTmp = _posRingBuffer[index];
                glm::quat qTmp = _rotRingBuffer[index];
                index = (_magRingIndex + _ringBack) % _ringSize;
                signal = _magRingBuffer[index];
                ret.translation = vTmp;
                ret.rotation = qTmp;

                #if WANT_DEBUG
                qDebug() << " Output: " << vTmp.x << "\t" << vTmp.y << "\t" << vTmp.z << "\t"
                         << "\t" << "Rotation: " << rot.w << "\t "<< rot.x << "\t " << rot.y << "\t " << rot.z;
                #endif
            }

        }


        uintptr_t len = _posBuffer.size();
        if (len > 0) {
            ret.translation = _posBuffer[len-1];
            _posBuffer.pop_back();
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

        len = _rotBuffer.size();
        if (len > 0) {
            ret.rotation = _rotBuffer[len-1];
            _rotBuffer.pop_back();
        }
        else {
            ret.rotation = rot;
        }

        #if WANT_DEBUG

        if (glm::dot(vTmp, vTmp) != 0.0f) {
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