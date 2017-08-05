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
           
                glm::vec3 ringBufferManager(glm::vec3 v, uint size) const;
                glm::quat ringBufferManager(glm::quat q, uint size) const;
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
                mutable bool _notZeroFlag { false };

                const uintptr_t getPosRingBufferSize() const { return _posRingBuffer.size(); }
                void setPosRingBuffer(glm::vec3 v, uintptr_t i) const { _posRingBuffer[i] = v; }
                void setPosRingBuffer(glm::vec3 v) const { _posRingBuffer.push_back(v); }
                const uintptr_t getPosRingIndex() const { return _posRingIndex; }
                void setPosRingIndex(uintptr_t i) const { _posRingIndex = i; }
                const uintptr_t getRingBack() const { return _ringBack;  }
                const glm::vec3 getPosRingBuffer(uintptr_t i) const { return _posRingBuffer[i]; }

    };

}


#endif
