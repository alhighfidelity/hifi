//
//  Created by Al Bernstein 2017/07/06
//  Copyright 2017 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_Controllers_Filters_High_Velocity_h
#define hifi_Controllers_Filters_High_Velocity_h

#define _USE_MATH_DEFINES

#include "../Filter.h"
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <QtCore/QDebug>
#include <vector>
#include <glm/gtx/string_cast.hpp>
#include <math.h>
#include <SimpleMovingAverage.h>
#include <chrono>
#include <ctime>

static const QString JSON_P_THRESHOLD = QStringLiteral("p_threshold");
static const QString JSON_Q_THRESHOLD = QStringLiteral("q_threshold");
static const QString JSON_SIZE = QStringLiteral("size");
static const QString JSON_P_WEIGHT = QStringLiteral("p_weight");
static const QString JSON_Q_WEIGHT = QStringLiteral("q_weight");
static const QString JSON_P_SAMPLES = QStringLiteral("p_samples");

namespace controller {
      
    class HighVelocityFilter : public Filter {
    REGISTER_FILTER_CLASS(HighVelocityFilter);
           
        public:
            HighVelocityFilter();
            HighVelocityFilter(float pThreshold, int pWeight, float qThreshold, int qWeight, int size);
            HighVelocityFilter(const HighVelocityFilter &other);
            
            virtual float apply(float value) const override { return value; }
            virtual Pose apply(Pose newPose) const override;
            virtual bool parseParameters(const QJsonValue& parameters) override;

            private:
           
                glm::vec3 diff(glm::vec3 v1, glm::vec3 v2) const;
                void pThreshold(glm::vec3 d_v,glm::vec3 v) const;
                void qThreshold(const float &signal, const float &threshold) const;
                void processPos(const int &k) const;
                void processRot() const;
                glm::quat  deltaQ(glm::quat qvec) const;
                float qDot(glm::quat q1, glm::quat q2) const;
                glm::quat unitVecAngle(glm::quat q) const;
                glm::quat qFlip(glm::quat q) const;
                float maxIndex(glm::vec3 v) const;
                glm::quat qFlipHemi(glm::quat q) const;
                std::vector<float> createPosStep(std::vector<float> data) const;
                std::vector <glm::quat> createRotStep() const;
                std::vector<float>  getPos(const int &i, std::vector<glm::vec3> v) const;
                void setPosOut(const int &k, std::vector<float> v) const;
                void setRotOut(std::vector<glm::quat> vq) const;
                void buildOutputArrays() const;
                glm::vec3 updatePosOut(glm::vec3 v) const;
                glm::quat updateRotOut(glm::quat q) const;
                Pose DataToPose(glm::vec3 pos, glm::quat rot) const;
                glm::vec3 ringBufferManager(const glm::vec3 &v,const size_t &size) const;
                glm::quat ringBufferManager(const glm::quat &q, const size_t &size) const;
                float ringBufferManager(const float &mag, const size_t &size) const;
                mutable std::vector<glm::vec3> _posOutput;
                mutable std::vector<glm::quat> _rotOutput;
                mutable std::vector<glm::vec3> _posRingBuffer;
                mutable std::vector<glm::quat> _rotRingBuffer;
                mutable std::vector<float> _magRingBuffer;
                const size_t _ringSize{ 11 };
                mutable size_t _ringBack{ 5 };
                mutable size_t _posRingIndex{ 5 };
                mutable size_t _rotRingIndex{ 5 };
                mutable size_t _magRingIndex{ 5 };
                float _pThresh{ 0.5f };
                size_t _n{ 0 };
                mutable int _pCount { 0 };
                mutable int _qCount{ 0 };
                float _qThresh { 0.5f };
                mutable int _pWeight{ 2 };
                mutable int _qWeight{ 2 };
                mutable glm::quat _qRef;
                mutable ThreadSafeMovingAverage<glm::vec3, 2> _pMvAvg;
                mutable ThreadSafeMovingAverage<glm::quat, 2> _qMvAvg;
                mutable float _deltaTime;
                mutable clock_t _currTime;
                mutable uint _numberSamples;
                mutable uint _pSamples;
                mutable glm::vec3 _posAvg;

    };

}


#endif
