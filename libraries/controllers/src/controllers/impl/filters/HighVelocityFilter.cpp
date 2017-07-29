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
        
        _pCount = 0;
        _qCount = 0;
        _pThresh = 0.0;
        _qThresh = 0.0f;
        _pWeight = 0.0f;
        _qWeight = 0.0f;
        _posOutput.clear();
        _pMvAvg.setWeight(_pWeight);
        _qMvAvg.setWeight(_qWeight);
        _posRingIndex = 0;
        _rotRingIndex = 0;
        _magRingIndex = 0;
        _ringBack = (_ringSize - 1) / 2;
        _currTime = std::clock();
        _numberSamples = 0;
        _callCount = 0;

        //qDebug() << " N = " << _posOutput.size();

        buildOutputArrays();
    }

    HighVelocityFilter::HighVelocityFilter(float pThreshold, int pWeight, float qThreshold, int qWeight, int size){

        _pThresh = pThreshold;
        _qThresh = qThreshold;
        _pCount = 0;
        _qCount = 0;
        _pWeight = pWeight;
        _qWeight = qWeight;
        _qRef = { 1.0f, 0.0f, 0.0f, 0.0f };

        _n = size;

        buildOutputArrays();

    }

    HighVelocityFilter::HighVelocityFilter(const HighVelocityFilter &other) {
       

        _pThresh = other._pThresh;
        _qThresh = other._qThresh;
        _n = other._n;
        _pCount = other._pCount;
        _qCount = other._qCount;

        _pWeight = other._pWeight;
        _qWeight = other._qWeight;
        _qRef = other._qRef;

    }

    HighVelocityFilter::~HighVelocityFilter() {
        _magRingBuffer.clear();
        _posRingBuffer.clear();
        _rotRingBuffer.clear();
    }


    glm::vec3 HighVelocityFilter::ringBufferManager(const glm::vec3 &v, const size_t &size) const {
    
        size_t len = _posRingBuffer.size();
        glm::vec3 ret = { 0.0f, 0.0f, 0.0f };

        if (len < size) {
            _posRingBuffer.push_back(v);
        }
        else {
            size_t index = (_posRingIndex + _ringBack) % size;
            ret = _posRingBuffer[index];
            _posRingBuffer[_posRingIndex] = v;
            _posRingIndex++;
            _posRingIndex = _posRingIndex % size;
        }

       
        //#if WANT_DEBUG

        size_t len1 = _magRingBuffer.size();

        qDebug() << " Pos Buffer: _posRingIndex = " << _posRingIndex << " length before = " << len << " length after = " << len1 << endl;
        for (size_t i = 0; i < len1; i++) {
            qDebug() << i << "\t" << _posRingBuffer[i].x << "\t" << _posRingBuffer[i].y << "\t" << _posRingBuffer[i].z << endl;
        }

        //#endif

        return ret;
}
       

    glm::quat HighVelocityFilter::ringBufferManager(const glm::quat &q, const size_t &size) const {

        size_t len = _rotRingBuffer.size();
        glm::quat ret = { 0.0f, 0.0f, 0.0f, 0.0f };

        if (len < size) {
            _rotRingBuffer.push_back(q);
        }
        else {
            ret = _rotRingBuffer[_rotRingIndex];
            _rotRingBuffer[_rotRingIndex] = q;
            _rotRingIndex++;
            _rotRingIndex = _rotRingIndex % size;
        }

        #if WANT_DEBUG
        // write out buffer
        size_t len1 = _rotRingBuffer.size();
        qDebug() << " Rot Buffer: _rotRingIndex = " << _rotRingIndex << " length before = " << len << " length after = " << len1 << endl;
        for (size_t i = 0; i < len1; i++) {
            qDebug() << i << "\t" << _rotRingBuffer[i] << endl;
        }
        #endif


        return ret;
    }


    float HighVelocityFilter::ringBufferManager(const float &mag, const size_t &size) const {

        size_t len = _magRingBuffer.size();
        float ret = 0.0f;

        if (len < size) {
            _magRingBuffer.push_back(mag);
        }
        else {
            size_t index = (_magRingIndex + _ringBack) % size;
            ret = _magRingBuffer[index];
            _magRingBuffer[_magRingIndex] = mag;
            _magRingIndex++;
            _magRingIndex = _magRingIndex % size;
        }

       
        
        //#if WANT_DEBUG
        // write out buffer
            size_t len1 = _magRingBuffer.size();
            qDebug() << " Mag Buffer: _magRingIndex = " << _magRingIndex << " length before = " << len << " length after = " << len1 << endl;
            for (size_t i = 0; i < len1; i++) {
                qDebug() << i << "\t" << _magRingBuffer[i] << endl;
            }
        //#endif

        return ret;
    }


    Pose HighVelocityFilter::apply(Pose newPose) const {

        Pose ret;

        glm::vec3 pos = newPose.getTranslation();
        glm::quat rot = newPose.getRotation();

       notZeroFlag = glm::dot(pos, pos) != 0.0f;


        // #if WANT_DEBUG
        if (glm::dot(pos, pos) != 0.0f) {
            qDebug() << " Filter Input: " << " " << newPose.getTranslation().x << " " << newPose.getTranslation().y << " " << newPose.getTranslation().z << " "
                << newPose.getRotation().w << " " << newPose.getRotation().x << " " << newPose.getRotation().y << " " << newPose.getRotation().z;
        }
        //#endif

        ret.translation = pos;
        ret.rotation = rot;

        if (glm::dot(pos, pos) != 0.0f) {

            float weight = 1.0f / (float)_pWeight;

            if (_numberSamples == 0) {
                _posAvg = { 0.0f, 0.0f, 0.0f };
            }
            else {
                _posAvg = pos * weight + (1.0f - weight)*_posAvg;
            }

            glm::vec3 avg = _posAvg;

            glm::vec3 dv = pos - avg;
            float dvMag = sqrtf(glm::dot(dv, dv));
            float signal = ringBufferManager(dvMag, _ringSize);
            glm::vec3 vTmp = ringBufferManager(pos, _ringSize);
            glm::quat qTmp = ringBufferManager(rot, _ringSize);

            _numberSamples++;

           // #if WANT_DEBUG
            if (glm::dot(vTmp, vTmp) != 0.0f) {
                qDebug() << " After Ring Buffer - Input: " << " " << vTmp.x << " " << vTmp.y << " " << vTmp.z << " "
                    << qTmp.w << " " << qTmp.x << " " << qTmp.y << " " << qTmp.z;
                     // " " << signal << endl;
                //qDebug() << "threshold:\t " << _pThresh << "_numberSamples:\t " << _numberSamples << "_pSamples:\t " << _pSamples <<endl;
                // #endif
            }

            if (_numberSamples >= _pSamples){
                size_t index = 0;

                if (signal >= _pThresh) {

                    index = _posRingIndex;
                    //glm::vec3 begin = _posRingBuffer[index];  // first
                    index = (_posRingIndex - 1) % _ringSize;
                    //glm::vec3 end = _posRingBuffer[index]; // last

                    // write out buffer

                    // #if WANT_DEBUG
                    qDebug() << " Load Position Buffer " << endl;
                   
                    // write out buffer before
                    qDebug() << " Pos Buffer Before: _ringIndex = " << _posRingIndex << endl;
                    size_t len = _posRingBuffer.size();
                    for (size_t i = 0; i < len; i++) {
                        qDebug() << i << "\t" << _posRingBuffer[i].x << "\t" << _posRingBuffer[i].y << "\t" << _posRingBuffer[i].z << endl;
                    }
                    // #endif

                    // set up step in ring buffer

                    // comment out for now

                    #if FILTER_OFF

                    _posRingBuffer[_posRingIndex] = begin;
                    size_t start = (_posRingIndex + 1) % _ringSize;

                    for (size_t i = start; i < start + _ringSize - 1; i++) {
                        index = i%_ringSize;
                        //#if WANT_DEBUG
                        qDebug() << "index = " << index << endl;
                        //#endif
                        _posRingBuffer[index] = end;
                    }
                   
                   // #if WANT_DEBUG
                    // write out buffer after
                    qDebug() << " Pos Buffer After: _ringIndex = " << _posRingIndex;
                    len = _posRingBuffer.size();
                    for (size_t i = 0; i < len; i++) {
                        qDebug() << i << "\t" << _posRingBuffer[i].x << "\t" << _posRingBuffer[i].y << "\t" << _posRingBuffer[i].z;
                    }

                   // #endif

                    // average over step function

                    avg = begin;

                    for (size_t i = start; i < start + _ringSize - 1; i++) {
                        index = i%_ringSize;
                       // #if WANT_DEBUG
                        qDebug() << "index = " << index << endl;
                       // #endif
                        avg = _posRingBuffer[index] * weight + (1.0f - weight)*avg;
                        _posRingBuffer[index] = avg;
                    }

                    // clear signal buffer

                    for (size_t i = 0; i < _ringSize; i++){
                        index = i%_ringSize;
                        _magRingBuffer[index] = 0.0f;
                    }

                    len = _posRingBuffer.size();
                    glm::vec3 tmp = _posRingBuffer[len-1];
                    len = _rotRingBuffer.size();
                    glm::quat q_tmp = _rotRingBuffer[len-1];

                    for (size_t i = 0; i < _ringSize; i++) {
                        glm::vec3 vTmp = ringBufferManager(tmp, _ringSize);
                        std::vector<glm::vec3>::iterator it = _posBuffer.begin();
                        _posBuffer.insert(it, vTmp);
                        glm::quat qTmp = ringBufferManager(q_tmp, _ringSize);
                        std::vector<glm::quat>::iterator it1 = _rotBuffer.begin();
                        _rotBuffer.insert(it1, qTmp);
                    }

                    #endif

                    len = _posRingBuffer.size();
                    glm::vec3 tmp = _posRingBuffer[len - 1];
                   // glm::vec3 vTmp = ringBufferManager(tmp, _ringSize);
                    glm::vec3 vTmp = _posRingBuffer[_posRingIndex];
                    std::vector<glm::vec3>::iterator it = _posBuffer.begin();
                    _posBuffer.insert(it, vTmp);

                    // #if WANT_DEBUG
                    // write out buffer after
                    qDebug() << " Pos Buffer After: _ringIndex = " << _posRingIndex << endl;
                    len = _posRingBuffer.size();
                    for (size_t i = 0; i < len; i++) {
                        qDebug() << i << "\t" << _posRingBuffer[i].x << "\t" << _posRingBuffer[i].y << "\t" << _posRingBuffer[i].z;
                    }
                    

                    // write out buffer after
                    qDebug() << " Mag Buffer After: _ringIndex = " << _magRingIndex << endl;

                    len = _magRingBuffer.size();
                    for (size_t i = 0; i < len; i++) {
                        qDebug() << i << "\t" << _magRingBuffer[i] << endl;
                    }
                    // #endif
                }
                else {
                    std::vector<glm::vec3>::iterator it = _posBuffer.begin();
                    _posBuffer.insert(it, vTmp);
                    std::vector<glm::quat>::iterator it1 = _rotBuffer.begin();
                    _rotBuffer.insert(it1, qTmp);
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


        size_t len = _posBuffer.size();
        if (len > 0) {
            ret.translation = _posBuffer[len - 1];
            _posBuffer.pop_back();
        }
        else {
            ret.translation = { 0.0f, 0.0f, 0.0f };
        }

        len = _rotBuffer.size();
        if (len > 0) {
            ret.rotation = _rotBuffer[len-1];
            _rotBuffer.pop_back();
        }
        else {
            ret.rotation = { 0.0f, 0.0f, 0.0f, 0.0f };
        }

        glm::vec3 vTmp = ret.getTranslation();
        glm::quat qTmp = ret.getRotation();
       
        if (glm::dot(vTmp, vTmp) != 0.0f) {
            qDebug() << " Filter Output: " << vTmp.x << " " << vTmp.y << " " << vTmp.z << " " << qTmp.w << " " << qTmp.x << " " << qTmp.y << " " << qTmp.z;
        }


        _callCount++;

        return ret;
    }


    glm::vec3 HighVelocityFilter::diff(glm::vec3 v1, glm::vec3 v2) const {
        
        glm::vec3 ret;

        ret = v2 - v1;
        return ret;
    }

    void HighVelocityFilter::pThreshold(glm::vec3 v,glm::vec3 pos) const {
        
        for (size_t i = 0; i < 3; i++){
            if (v[i] > _pThresh) {
                processPos(i);
            }
        }
    }

    void HighVelocityFilter::qThreshold(const float &signal, const float &threshold) const {

        if (signal > threshold){
            processRot();
        }
    }

    glm::quat HighVelocityFilter::deltaQ(glm::quat q) const {

        if (!_rotOutput.empty()) {
            _qRef = _rotOutput[0];
        }

        glm::quat qtmp = q;
        qtmp = qFlipHemi(qtmp);

        _qMvAvg.addSample(qtmp);
        glm::quat avg = _qMvAvg.getAverage();
        glm::quat qI = glm::inverse(avg);
        glm::quat ret = qtmp * qI;
        ret = glm::normalize(ret);

        return ret;
    }

    glm::quat HighVelocityFilter::qFlipHemi(glm::quat q) const {

        glm::quat ret = q;

        if (glm::dot(_qRef, q) < 0) {
            ret = -ret;
        }

        return ret;
    }

    float HighVelocityFilter::qDot(glm::quat q1, glm::quat q2) const {

        glm::quat qtmp1 = unitVecAngle(q1);
        glm::quat qtmp2 = unitVecAngle(q2);
        qtmp1 = qFlip(qtmp1);
        qtmp2 = qFlip(qtmp2);
        float th1 = qtmp1.w;
        float th2 = qtmp2.w;
        glm::vec3 v1 = { qtmp1.x, qtmp1.y, qtmp1.z };
        glm::vec3 v2 = { qtmp2.x, qtmp2.y, qtmp2.z };
        float ret = th1*th2*glm::dot(v1, v2);

        return ret;
    }

    glm::quat HighVelocityFilter::qFlip(glm::quat q) const {

        glm::vec3 v = { q.x, q.y, q.z };
        glm::quat qtmp = q;

        float val = maxIndex(v);
        if (val < 0) {
            qtmp = -qtmp;
        }

        return qtmp;
    }

    float  HighVelocityFilter::maxIndex(glm::vec3 v)  const {

        float max = -1000.0f;
        size_t indx = 0;

        for (size_t i = 0; i < 3; i++) {
            if (fabsf(v[i]) > max) {
                max = fabsf(v[i]);
                indx = i;
            }
        }

        return v[indx];
    }

    glm::quat HighVelocityFilter::unitVecAngle(glm::quat q) const {

        glm::quat ret = { 0.0f, 0.0f, 0.0f, 0.0f };
        float psi = 2.0f*acosf(q.w);
        float s = sinf(psi / 2.0f);

        if (s == 0) {
            if (fabsf(fmodf(psi, (float)M_PI)) < 1e-6f){
                ret.x = 0.0f;
                ret.y = 0.0f;
                ret.z = 1.0f;
                ret.w = 0.0f;
                return ret;
            }
            else {
                ret.x = 0.0f;
                ret.y = 0.0f;
                ret.z = -1.0f;
                ret.w = 0.0;

                return ret;
            }
        }

        glm::vec3 uc;

        uc.x = q.x / s;
        uc.y = q.y / s;
        uc.z = q.z / s;
       
        ret.x = uc.x;
        ret.y = uc.y;
        ret.z = uc.z;
        ret.w = psi;

        return ret;
    }


    void HighVelocityFilter::processPos(const int &k ) const {
        
        ThreadSafeMovingAverage<float, 2> mvAvg;
        std::vector<float> pos = getPos(k, _posOutput);
        std::vector <float> vPos = createPosStep(pos);
        std::vector<float> avg;

        for (size_t i = 0; i < vPos.size(); i++) {
            mvAvg.addSample(vPos[i]);
            avg.push_back(mvAvg.getAverage());
        }
        setPosOut(k, avg);
    }


    void HighVelocityFilter::processRot() const {
        
        ThreadSafeMovingAverage<glm::quat, 2> mvAvg;
        std::vector <glm::quat> vRot = createRotStep();
      
        std::vector<glm::quat> avg;
        
        for (size_t i = 0; i < vRot.size(); i++) {
            mvAvg.addSample(vRot[i]);
            avg.push_back(mvAvg.getAverage());
        }

        setRotOut(avg);
    }



    std::vector <glm::quat>  HighVelocityFilter::createRotStep() const {

        std::vector<glm::quat> ret;

        ret.push_back(_rotOutput[0]);

        for (size_t i = 1; i < _n; i++) {
            ret.push_back(_rotOutput[_n - 1]);
        }

        return ret;
    }


    std::vector<float> HighVelocityFilter::createPosStep(std::vector<float> data) const {
         
        std::vector<float> ret;
        size_t N = data.size();

        if (N == 0) {
            return ret;
        }
       
        ret.push_back(data[0]);

        for (size_t i = 1; i < N; i++) {
            ret.push_back(data[N-1]);
        }

        return ret;

    }

    std::vector<float>  HighVelocityFilter::getPos(const int &k, std::vector<glm::vec3> v) const {

        std::vector<float> ret;

        for (size_t i = 0; i < v.size(); i++) {
            ret.push_back(v[i][k]);
        }

        return ret;
    }

    void  HighVelocityFilter::setPosOut(const int &k, std::vector<float> v) const {

        for (size_t i = 0; i < v.size(); i++) {
            _posOutput[i][k] = v[i];
        }

    }

    void  HighVelocityFilter::setRotOut(std::vector<glm::quat> vq) const {

        for (size_t i = 0; i < vq.size(); i++) {
            _rotOutput[i] = vq[i];
        }
    }




    void HighVelocityFilter::buildOutputArrays() const {
        
        _posOutput.clear();
        _rotOutput.clear();
        glm::vec3 vtmp = { 0.0f, 0.0f, 0.0f };
        glm::quat qtmp = { 0.0f, 0.0f, 0.0f, 0.0f };
       
        for (size_t i = 0; i < _n; i++) {
            _posOutput.push_back(vtmp);
            _rotOutput.push_back(qtmp);
        }

    }

    glm::vec3 HighVelocityFilter::updatePosOut(glm::vec3 v) const {

        glm::vec3 ret;
        
        qDebug() << "Input: N = " << _posOutput.size() << " " << v.x << " " << v.y << " " << v.z;

        if (_posOutput.empty()){
            ret = v;
            _posOutput.push_back(v);
            _pCount++;
           
            return ret;
        }

        size_t N = _posOutput.size();

        if (N  < _n) {
            ret = _posOutput[0];
            _posOutput.push_back(v);
            _pCount++;
            return ret;
        }

        // N should be equal to _n

        ret = _posOutput[0];
        std::vector<glm::vec3>::iterator it = _posOutput.begin();
        _posOutput.erase(it);
        _posOutput.push_back(v);

        qDebug() << "Output: N = " << _posOutput.size() << " " << ret.x << " " << ret.y << " " << ret.z;

       return ret;
    }

    glm::quat HighVelocityFilter::updateRotOut(glm::quat q) const {
        
        glm::quat ret;
       

        if (_rotOutput.empty()){
            ret = q;
            _rotOutput.push_back(q);
            _qCount++;
            return ret;
        }

        size_t N = _rotOutput.size();

        if (N  < _n) {
            ret = _rotOutput[0];
            _rotOutput.push_back(q);
            _qCount++;
            return ret;
        }

        // N should be equal to _n

        ret = _rotOutput[0];
        std::vector<glm::quat>::iterator it = _rotOutput.begin();
        _rotOutput.erase(it);
        _rotOutput.push_back(q);

        return ret;
    }

    bool HighVelocityFilter::parseParameters(const QJsonValue& parameters) {

        if (parameters.isObject()) {
            auto obj = parameters.toObject();
            if (obj.contains(JSON_P_THRESHOLD) && obj.contains(JSON_Q_THRESHOLD) && obj.contains(JSON_SIZE) && 
                obj.contains(JSON_P_WEIGHT) && obj.contains(JSON_Q_WEIGHT) && obj.contains(JSON_P_SAMPLES)) {
                _pThresh = obj[JSON_P_THRESHOLD].toDouble();
                _pWeight = obj[JSON_P_WEIGHT].toDouble();
                _n = obj[JSON_SIZE].toInt();
                _qThresh = obj[JSON_Q_THRESHOLD].toInt();
                _qWeight = obj[JSON_Q_WEIGHT].toInt();
                _pSamples = obj[JSON_P_SAMPLES].toInt();
                return true;
            }
        }
        return false;
    }
}