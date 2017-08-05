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
#include <chrono>
#include <ctime>

static const QString JSON_P_THRESHOLD = QStringLiteral("p_threshold");
static const QString JSON_SIZE = QStringLiteral("size");
static const QString JSON_P_WEIGHT = QStringLiteral("p_weight");
static const QString JSON_Q_WEIGHT = QStringLiteral("q_weight");
static const QString JSON_P_SAMPLES = QStringLiteral("p_samples");

namespace controller {
      
    class HighVelocityFilter : public Filter {
        REGISTER_FILTER_CLASS(HighVelocityFilter);

    public:
        HighVelocityFilter();
        HighVelocityFilter(float pThreshold, float pWeight, float qThreshold, float qWeight, uint size);
        HighVelocityFilter(const HighVelocityFilter &other);
        ~HighVelocityFilter();

        virtual float apply(float value) const override { return value; }
        virtual Pose apply(Pose newPose) const override;
        virtual bool parseParameters(const QJsonValue& parameters) override;

    private:

        glm::vec3 ringBufferManager(glm::vec3 v, uintptr_t size) const;
        glm::quat ringBufferManager(glm::quat q, uintptr_t size) const;
        float ringBufferManager(float mag, uintptr_t size) const;
        mutable std::vector<glm::vec3> _posRingBuffer;
        mutable std::vector<glm::quat> _rotRingBuffer;
        mutable std::vector<float> _magRingBuffer;
        mutable std::vector<glm::vec3> _posBuffer;
        mutable std::vector<glm::quat> _rotBuffer;
        mutable uint _ringSize{ 11 };
        mutable uint _ringBack{ 5 };
        mutable uint _posRingIndex{ 5 };
        mutable uint _rotRingIndex{ 5 };
        mutable uint _magRingIndex{ 5 };
        float _pThresh{ 0.5f };
        mutable float _pWeight{ 0.5f };
        mutable float _qWeight{ 0.5f };
        mutable uint _numberSamples;
        mutable uint _avgLength;
        mutable glm::vec3 _posAvg;
        mutable bool _notZeroFlag{ false };

        uintptr_t getPosRingBufferSize() const { return _posRingBuffer.size(); }
        uintptr_t getRotRingBufferSize() const { return _rotRingBuffer.size(); }
        uintptr_t getMagRingBufferSize() const { return _magRingBuffer.size(); }
        uintptr_t getPosRingIndex() const { return _posRingIndex; }
        uintptr_t getRotRingIndex() const { return _rotRingIndex; }
        uintptr_t getMagRingIndex() const { return _magRingIndex; }
        uintptr_t getRingBack() const { return _ringBack; }
        glm::vec3 getPosRingBuffer(uintptr_t i) const { return _posRingBuffer[i]; }
        glm::quat getRotRingBuffer(uintptr_t i) const { return _rotRingBuffer[i]; }
        float getMagRingBuffer(uintptr_t i) const { return _magRingBuffer[i]; }
        uintptr_t getNumberSamples() const { return _numberSamples;  }
        uintptr_t getAvgLength() const { return _avgLength; }
        float getPosThreshold() const { return _pThresh; }
        glm::vec3 getPosAverage() const { return _posAvg; }
        float getPosWeight() const { return _pWeight;  }
        bool getNotZeroFlag() const { return _notZeroFlag; }
        uintptr_t getRingSize() const { return _ringSize; }
        float getRotWeight() const { return _qWeight; }
        std::vector<glm::vec3>::iterator getPosBufferBegin() const { return _posBuffer.begin(); }
        std::vector<glm::quat>::iterator getRotBufferBegin() const { return _rotBuffer.begin(); }
        uintptr_t getPosBufferSize() const { return _posBuffer.size(); }
        glm::vec3 getPosBuffer(uintptr_t i) const { return _posBuffer[i]; }
        void posBufferPop() const { _posBuffer.pop_back(); }
        uintptr_t getRotationBufferSize() const { return _rotBuffer.size(); }
        glm::quat getRotBuffer(uintptr_t i) const { return _rotBuffer[i]; }
        void rotBufferPop() const { _rotBuffer.pop_back(); }


        void setPosRingBuffer(glm::vec3 v, uintptr_t i) const { _posRingBuffer[i] = v; }
        void setPosRingBuffer(glm::vec3 v) const { _posRingBuffer.push_back(v); }
        void setRotRingBuffer(glm::quat q) const { _rotRingBuffer.push_back(q); }
        void setRotRingBuffer(glm::quat q, uintptr_t i) const { _rotRingBuffer[i] = q; }
        void setMagRingBuffer(float mag) const { _magRingBuffer.push_back(mag); }
        void setMagRingBuffer(float mag, uintptr_t i) const { _magRingBuffer[i] = mag; }
        void setPosRingIndex(uintptr_t i) const { _posRingIndex = i; }
        void setRotRingIndex(uintptr_t i) const { _rotRingIndex = i; }
        void setMagRingIndex(uintptr_t i) const { _magRingIndex = i; }
        void setPosAverage(glm::vec3 avg) const { _posAvg = avg; }
        void setNotZeroFlag(bool flag) const { _notZeroFlag = flag; }
        void setPosBuffer(std::vector<glm::vec3>::iterator it, glm::vec3 pos) const { _posBuffer.insert(it, pos); }
        void setRotBuffer(std::vector<glm::quat>::iterator it, glm::quat rot) const { _rotBuffer.insert(it, rot); }
        void setNumSamples(uintptr_t num) const { _numberSamples = num; }

    };
}


#endif
