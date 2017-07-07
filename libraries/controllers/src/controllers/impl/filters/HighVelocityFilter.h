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
#include <vector>

static const QString JSON_P_THRESHOLD = QStringLiteral("p_threshold");
static const QString JSON_Q_THRESHOLD = QStringLiteral("q_threshold");
static const QString JSON_SIZE = QStringLiteral("size");
static const QString JSON_P_WEIGHT = QStringLiteral("p_weight");
static const QString JSON_Q_WEIGHT = QStringLiteral("q_weight");


namespace controller {

    class HighVelocityFilter : public Filter {
    REGISTER_FILTER_CLASS(HighVelocityFilter);
           
        public:
            HighVelocityFilter();
            
            virtual Pose apply(Pose newPose) const;
            virtual bool parseParameters(const QJsonValue& parameters) override;

            private:
           
                glm::vec3 diff(glm::vec3 v1, glm::vec3 v2) const;
                void pThreshold(glm::vec3 d_v,glm::vec3 v) const;
                void HighVelocityFilter::qThreshold(const float &signal, const float &threshold) const;
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
            
                mutable std::vector<glm::vec3> _posOutput;
                mutable std::vector<glm::quat> _rotOutput;
                float _p_thresh{ 0.5f };
                int _N;
                mutable int _p_count { 0 };
                mutable int _q_count{ 0 };
                float _q_thresh { 0.5f };
                const int _p_weight{ 2 };
                const int _q_weight{ 2 };
                mutable glm::quat _q_ref;
    };

}


#endif
