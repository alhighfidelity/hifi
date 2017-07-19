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
        _currTime = std::clock();
        _numberSamples = 0;

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

    glm::vec3 HighVelocityFilter::ringBufferManager(const glm::vec3 &v, const size_t &size) const {
        
        size_t len = _posRingBuffer.size();
        glm::vec3 ret = { 0.0f, 0.0f, 0.0f };

        if (len < size + 1) {
            _posRingBuffer.push_back(v);
        }
        else {
            ret = _posRingBuffer[_posRingIndex];
            _posRingBuffer[_posRingIndex] = v;
            _posRingIndex++;
            _posRingIndex = _posRingIndex % size;
        }

        // write out buffer
        /* qDebug() << " Pos Buffer: _posRingIndex = " << _posRingIndex;
        for (int i = 0; i < len; i++) {
            qDebug() << i << "\t" << _posRingBuffer[i].x << "\t" << _posRingBuffer[i].y << "\t" << _posRingBuffer[i].z;
        } */

        return ret;
    }

    glm::quat HighVelocityFilter::ringBufferManager(const glm::quat &q, const size_t &size) const {

        size_t len = _rotRingBuffer.size();
        glm::quat ret = { 0.0f, 0.0f, 0.0f, 0.0f };

        if (len < size + 1) {
            _rotRingBuffer.push_back(q);
        }
        else {
            ret = _rotRingBuffer[_rotRingIndex];
            _rotRingBuffer[_rotRingIndex] = q;
            _rotRingIndex++;
            _rotRingIndex = _rotRingIndex % size;
        }

        return ret;
    }

    float HighVelocityFilter::ringBufferManager(const float &mag, const size_t &size) const {

        size_t len = _magRingBuffer.size();
        float ret = 0.0f;

        if (len < size + 1) {
            _magRingBuffer.push_back(mag);
        }
        else {
            ret = _magRingBuffer[_magRingIndex];
            _magRingBuffer[_magRingIndex] = mag;
            _magRingIndex++;
            _magRingIndex = _magRingIndex % size;
        }
        
        // write out buffer
        /*qDebug() << " Mag Buffer: _ringIndex = " << _magRingIndex;
        for (int i = 0; i < len; i++) {
            qDebug() << i << "\t" << _magRingBuffer[i];
        } */


        return ret;

    }


    Pose HighVelocityFilter::apply(Pose newPose) const {

        clock_t tmp = std::clock();
        _deltaTime = (tmp - _currTime)*1.0e-2;
        _currTime = tmp;

        glm::vec3 pos = newPose.getTranslation();
        glm::quat rot = newPose.getRotation();
     
        Pose ret;

        ret.translation = pos;
        ret.rotation = rot;

        if (glm::dot(pos, pos) != 0) {
            //_pMvAvg.addSample(pos);
            //glm::vec3 avg = _pMvAvg.getAverage();

            float weight = 1.0f /(float) _pWeight;

            if (_numberSamples == 0) {
                _posAvg = { 0.0f, 0.0f, 0.0f };
            }
            else {
                _posAvg = pos * weight + (1.0f - weight)*_posAvg;
            }
   
            glm::vec3 avg = _posAvg;

            glm::vec3 dv = pos - avg;
            float dvMag = sqrtf(glm::dot(dv, dv));
            float dvTmp1 = ringBufferManager(dvMag, _ringSize);
            glm::vec3 vTmp = ringBufferManager(pos, _ringSize);

            qDebug() << " Input: " << "\t" << vTmp.x << "\t" << vTmp.y << "\t" << vTmp.z << "\t" << dvTmp1;
            _numberSamples++;

          /* if (_numberSamples > _pSamples){
                glm::vec3 dv = pos - avg;
                glm::vec3 vTmp = ringBufferManager(pos, _ringSize);
                float dvMag = sqrtf(glm::dot(dv, dv));
                float dvTmp1 = ringBufferManager(dvMag, _ringSize);
                float dvTmp2 = _magRingBuffer[0];
                float d_a = fabsf(dvTmp1 - dvTmp2)/_deltaTime;
                glm::quat qTmp = ringBufferManager(rot, _ringSize);
                 
                if (d_a > _pThresh) {
                     
                    size_t n = _posRingBuffer.size();
                    if (n - 1 >= 0) {
                        glm::vec3 vTmp = _posRingBuffer[n - 1];
                    }

                    for (int i = 1; i < n; i++) {
                        _posRingBuffer[i] = vTmp;
                    }

                    ThreadSafeMovingAverage<glm::vec3, 2> pMvAvg;
                    pMvAvg.addSample(_posRingBuffer[0]);

                    for (int i = 1; i < n; i++) {
                        pMvAvg.addSample(_posRingBuffer[i]);
                        _posRingBuffer[i] = pMvAvg.getAverage();
                    }
                }
                 
                vTmp = _posRingBuffer[0];
                //qTmp = _rotRingBuffer[0];
                ret.translation = vTmp;
                ret.rotation = qTmp;

                
                        
             } */
        }
        
       // quat rot = newPose.getRotation();
       
       // qDebug() << "Input: " << pos.x << " " << pos.y << " " << pos.z
       //     << " " << rot.w << " " << rot.x << " " << rot.y << " " << rot.z << endl;
  
        //size_t N = _posOutput.size();

        // if ( N > 0) {

            // print input parameters

            //qDebug() << "parameters" << "\t" << "_pThresh = " << _pThresh << "\t" << "_pWeight = " << _pWeight;


            // translation processing

            //glm::vec3 d_pos = diff(_posOutput[_pCount-1], pos);
            //d_pos = abs(d_pos);

            //glm::vec3 v1 = _posOutput[_pCount - 1];
            //glm::vec3 v2 = pos;

            //qDebug() << "pCount - 1 " << _pCount - 1 << " v1 = \t" << v1.x << "\t" << v1.y << "\t" << v1.z
            //                          << "\t" << "v2 = " << v2.x << "\t" << v2.y << "\t" << v2.z
            //                          << " d_pos: \t" << d_pos.x << "\t" << d_pos.y << "\t" << d_pos.z;
           
            //qDebug() << "d_pos_z\t" << d_pos.z;


            // << " " << qOut.w << " " << qOut.x << " " << qOut.y << " " << qOut.z << endl;

            //qDebug() << "HighVelocityFilter";
            //qDebug() << "N = " << N << " d_pos  = " << glm::to_string(d_pos).c_str();
            //qDebug() << " pos  = " << glm::to_string(pos).c_str() << "_posOutput[_pCount-1] = " << glm::to_string(_posOutput[_pCount - 1]).c_str()
            //    << " rotation = " << glm::to_string(rot).c_str();
           // pThreshold(d_pos,pos);

           
            // rotation processing

            //glm::quat dQ = deltaQ(rot);
            //float q_dot = qDot(_rotOutput[_pCount - 1], dQ);
            //qThreshold(q_dot, _qThresh);

        //}
      
        // set up output

        //glm::vec3 pOut = updatePosOut(pos);
        //qDebug() << "N = " << N << " pOut  = " << glm::to_string(pOut).c_str();

        //glm::quat qOut = updateRotOut(rot);

        
       //Pose ret = DataToPose(pOut, qOut);

       // glm::vec3 pos = newPose.getTranslation();
       // glm::quat rot = newPose.getRotation();

       

       


        // print output values

//       qDebug() << "Output: " << pOut.x << " " << pOut.y << " " << pOut.z
//           << " " << qOut.w << " " << qOut.x << " " << qOut.y << " " << qOut.z << endl;
     
       return ret;
    }


    Pose HighVelocityFilter::DataToPose(glm::vec3 pos, glm::quat rot) const {

        Pose ret;

        // set position and rotation data

        ret.translation = pos;
        ret.rotation = rot;

        return ret;
    }





    glm::vec3 HighVelocityFilter::diff(glm::vec3 v1, glm::vec3 v2) const {
        
        glm::vec3 ret;

        ret = v2 - v1;
        return ret;
    }

    void HighVelocityFilter::pThreshold(glm::vec3 v,glm::vec3 pos) const {
        
        for (int i = 0; i < 3; i++){
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

        float max = -1000.0;
        int indx = 0;

        for (int i = 0; i < 3; i++) {
            if (std::fabsf(v[i]) > max) {
                max = std::fabsf(v[i]);
                indx = i;
            }
        }

        return v[indx];
    }

    glm::quat HighVelocityFilter::unitVecAngle(glm::quat q) const {

        glm::quat ret = { 0.0f, 0.0f, 0.0f, 0.0f };
        float psi = 2.0*acosf(q.w);
        float s = sinf(psi / 2.0);

        if (s == 0) {
            if (std::fabsf(std::fmodf(psi, (float)M_PI)) < 1e-6){
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