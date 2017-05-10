//  KinectPlugin.h
//
//  Created by Brad Hefta-Gaub on 2016/12/7
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_KinectPlugin_h
#define hifi_KinectPlugin_h

#ifdef HAVE_KINECT
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers
#endif

// Windows Header Files
#include <windows.h>

#include <Shlobj.h>

// Kinect Header files
#include <Kinect.h>
#include <SimpleMovingAverage.h>

// Safe release for interfaces
template<class Interface> inline void SafeRelease(Interface *& pInterfaceToRelease) {
    if (pInterfaceToRelease != NULL) {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}
#endif

#include <controllers/InputDevice.h>
#include <controllers/StandardControls.h>
#include <plugins/InputPlugin.h>

 
// Handles interaction with the Kinect SDK
class KinectPlugin : public InputPlugin {
    Q_OBJECT
public:
    bool isHandController() const override;

    // Plugin functions
    virtual void init() override;
    virtual bool isSupported() const override;
    virtual const QString getName() const override { return NAME; }
    const QString getID() const override { return KINECT_ID_STRING; }

    virtual bool activate() override;
    virtual void deactivate() override;

    virtual void pluginFocusOutEvent() override { _inputDevice->focusOutEvent(); }
    virtual void pluginUpdate(float deltaTime, const controller::InputCalibrationData& inputCalibrationData) override;

    virtual void saveSettings() const override;
    virtual void loadSettings() override;

protected:



    struct KinectCalTrans {
        glm::quat position;
        glm::quat orientation;

    };

    struct KinectJointAvg {

        GenericMovingAverage<glm::vec3, 2> positionAvg;
        GenericMovingAverage<glm::quat, 2> orientationAvg;
    };

    ThreadSafeMovingAverage<glm::quat, 2> _RightHandOrientationAverage;
    ThreadSafeMovingAverage<glm::quat, 2> _LeftHandOrientationAverage;


    struct localBasis {
        glm::vec3 x;
        glm::vec3 y;
        glm::vec3 z;
        glm::vec3 hips;
    };

    struct KinectJoint {
        glm::vec3 position;
        glm::quat orientation;
    };


    class InputDevice : public controller::InputDevice {
    public:
        friend class KinectPlugin;

        // variables for calibaration
        
        // averaged joint data for calibration
        std::vector<KinectJointAvg> _avg_joints;
        std::vector<KinectCalTrans> _cal_targets;
        std::vector<KinectCalTrans> _cal_trans;

        localBasis _localBasis;
        
        mutable bool _calibrated{ false };
        mutable int _AvgSamples{ 0 };
        mutable std::mutex _lock;
        mutable bool _debug{ false };
        

        InputDevice() : controller::InputDevice("Kinect") {}

        // Device functions
        virtual controller::Input::NamedVector getAvailableInputs() const override;
        virtual QString getDefaultMappingConfig() const override;
        virtual void update(float deltaTime, const controller::InputCalibrationData& inputCalibrationData) override {};
        virtual void focusOutEvent() override {};

        void update(float deltaTime, const controller::InputCalibrationData& inputCalibrationData, 
        const std::vector<KinectJoint>& joints, const std::vector<KinectJoint>& prevJoints);
        void averageJoints(const KinectJoint &joints, const size_t &i);
        void buildAverageJoints();
        void deleteAverageJoints();
        void calculateCalibration();
        void calculateTransforms(const int &i);
        void calcCalibrationTargets();
        void applyTransform(const size_t &i, float deltaTime, const KinectJoint &joint,
            const KinectJoint &prevJoints, const controller::InputCalibrationData& inputCalibrationData);

        const glm::vec3  applyPos(const size_t &i, const KinectJoint & joint);
        const glm::quat  applyRot(const size_t &i, const KinectJoint & joint);
        KinectJoint TestTPose(size_t i);
        void TestCalibration();
        bool InJointSet(const size_t &i);
        void printJoints(const KinectJoint &joint, const JointType &jointType);
        void printJoints(const Joint &joint, const JointType &jointType, glm::vec3 jointPosition, glm::quat jointOrientation);
        void printJointAvg();
        void printPoseStateMap(const size_t &i);
        KinectJoint getJointAverage(const size_t &i);
        KinectJoint testTranslation(const KinectJoint &joint, glm::vec3 deltaV);
        void calculatDifferenceAverages();
        void  setLocalBasis();
        void updateLocalBasis();
        glm::vec3 transformLocalBasis(glm::vec3 pos);
        void clearState();
    };

    std::shared_ptr<InputDevice> _inputDevice { std::make_shared<InputDevice>() };
    static const char* NAME;
    static const char* KINECT_ID_STRING;

    bool _enabled { false };
    mutable bool _initialized { false };
    InputDevice _input;
   
    // copy of data directly from the KinectDataReader SDK
    std::vector<KinectJoint> _joints;

    // one frame old copy of _joints, used to caluclate angular and linear velocity.
    std::vector<KinectJoint> _prevJoints;


    // Kinect SDK related items...

    bool initializeDefaultSensor() const;
    void updateBody();

#ifdef HAVE_KINECT
    void ProcessBody(INT64 time, int bodyCount, IBody** bodies);

    // Current Kinect
    mutable IKinectSensor* _kinectSensor { nullptr };
    mutable ICoordinateMapper* _coordinateMapper { nullptr };

    // Body reader
    mutable IBodyFrameReader* _bodyFrameReader { nullptr };
#endif

};

#endif // hifi_KinectPlugin_h