//
//  KinectPlugin.cpp
//
//  Created by Brad Hefta-Gaub on 2016/12/7
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//


#include "KinectPlugin.h"

#include <controllers/UserInputMapper.h>
#include <QLoggingCategory>
#include <PathUtils.h>
#include <DebugDraw.h>
#include <cassert>
#include <NumericalConstants.h>
#include <StreamUtils.h>
#include <Preferences.h>
#include <SettingHandle.h>
#include <QTime>

Q_DECLARE_LOGGING_CATEGORY(inputplugins)
Q_LOGGING_CATEGORY(inputplugins, "hifi.inputplugins")

const char* KinectPlugin::NAME = "Kinect";
const char* KinectPlugin::KINECT_ID_STRING = "Kinect";

QStringList kinectJointNames = {
    "SpineBase",
    "SpineMid",
    "Neck",
    "Head",
    "ShoulderLeft",
    "ElbowLeft",
    "WristLeft",
    "HandLeft",
    "ShoulderRight",
    "ElbowRight",
    "WristRight",
    "HandRight",
    "HipLeft",
    "KneeLeft",
    "AnkleLeft",
    "FootLeft",
    "HipRight",
    "KneeRight",
    "AnkleRight",
    "FootRight",
    "SpineShoulder",
    "HandTipLeft",
    "ThumbLeft",
    "HandTipRight",
    "ThumbRight"
};

const bool DEFAULT_ENABLED = false;

enum KinectJointIndex {
    SpineBase = 0,
    SpineMid,
    Neck,
    Head,

    ShoulderLeft,
    ElbowLeft,
    WristLeft,
    HandLeft,

    ShoulderRight,
    ElbowRight,
    WristRight,
    HandRight,

    HipLeft,
    KneeLeft,
    AnkleLeft,
    FootLeft,

    HipRight,
    KneeRight,
    AnkleRight,
    FootRight,

    SpineShoulder,

    HandTipLeft,
    ThumbLeft,

    HandTipRight,
    ThumbRight,

    Size
};

#define UNKNOWN_JOINT (controller::StandardPoseChannel)0 

static controller::StandardPoseChannel KinectJointIndexToPoseIndexMap[KinectJointIndex::Size] = {
    controller::HIPS,
    controller::SPINE,
    controller::NECK,
    controller::HEAD,

    controller::LEFT_SHOULDER,
    controller::LEFT_ARM,
    controller::LEFT_FORE_ARM,
    controller::LEFT_HAND,

    controller::RIGHT_SHOULDER,
    controller::RIGHT_ARM,
    controller::RIGHT_FORE_ARM,
    controller::RIGHT_HAND,

    controller::LEFT_UP_LEG,   // hip socket
    controller::LEFT_LEG,      // knee?
    UNKNOWN_JOINT,              // ????
    controller::LEFT_FOOT,     // ankle?
    controller::RIGHT_UP_LEG,   // hip socket
    controller::RIGHT_LEG,      // knee?
    UNKNOWN_JOINT,              // ????
    controller::RIGHT_FOOT,     // ankle?

    UNKNOWN_JOINT, /* SpineShoulder */

    controller::LEFT_HAND_INDEX4,
    controller::LEFT_HAND_THUMB4,

    controller::RIGHT_HAND_INDEX4,
    controller::RIGHT_HAND_THUMB4,
};

// in rig frame
static glm::vec3 rightHandThumb1DefaultAbsTranslation(-2.155500650405884, -0.7610001564025879, 2.685631036758423);
static glm::vec3 leftHandThumb1DefaultAbsTranslation(2.1555817127227783, -0.7603635787963867, 2.6856393814086914);

static controller::StandardPoseChannel KinectJointIndexToPoseIndex(KinectJointIndex i) {
    assert(i >= 0 && i < KinectJointIndex::Size);
    if (i >= 0 && i < KinectJointIndex::Size) {
        return KinectJointIndexToPoseIndexMap[i];
    } else {
        return UNKNOWN_JOINT; // not sure what to do here, but don't crash!
    }
}

QStringList controllerJointNames = {
    "Hips",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "Spine",
    "Spine1",
    "Spine2",
    "Spine3",
    "Neck",
    "Head",
    "RightShoulder",
    "RightArm",
    "RightForeArm",
    "RightHand",
    "RightHandThumb1",
    "RightHandThumb2",
    "RightHandThumb3",
    "RightHandThumb4",
    "RightHandIndex1",
    "RightHandIndex2",
    "RightHandIndex3",
    "RightHandIndex4",
    "RightHandMiddle1",
    "RightHandMiddle2",
    "RightHandMiddle3",
    "RightHandMiddle4",
    "RightHandRing1",
    "RightHandRing2",
    "RightHandRing3",
    "RightHandRing4",
    "RightHandPinky1",
    "RightHandPinky2",
    "RightHandPinky3",
    "RightHandPinky4",
    "LeftShoulder",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
    "LeftHandThumb1",
    "LeftHandThumb2",
    "LeftHandThumb3",
    "LeftHandThumb4",
    "LeftHandIndex1",
    "LeftHandIndex2",
    "LeftHandIndex3",
    "LeftHandIndex4",
    "LeftHandMiddle1",
    "LeftHandMiddle2",
    "LeftHandMiddle3",
    "LeftHandMiddle4",
    "LeftHandRing1",
    "LeftHandRing2",
    "LeftHandRing3",
    "LeftHandRing4",
    "LeftHandPinky1",
    "LeftHandPinky2",
    "LeftHandPinky3",
    "LeftHandPinky4"
};

static const char* getControllerJointName(controller::StandardPoseChannel i) {
    if (i >= 0 && i < controller::NUM_STANDARD_POSES) {
        return qPrintable(controllerJointNames[i]);
    }
    return "unknown";
}

//
// KinectPlugin
//
void KinectPlugin::init() {
    loadSettings();

    auto preferences = DependencyManager::get<Preferences>();
    static const QString KINECT_PLUGIN { "Kinect" };
    {
        auto getter = [this]()->bool { return _enabled; };
        auto setter = [this](bool value) { 
            _enabled = value; 
            saveSettings(); 
            if (!_enabled) {
                auto userInputMapper = DependencyManager::get<controller::UserInputMapper>();
                userInputMapper->withLock([&, this]() {
                    _inputDevice->clearState();
                });
            }
        };
        auto preference = new CheckPreference(KINECT_PLUGIN, "Enabled", getter, setter);
        preferences->addPreference(preference);
    }
    {
        auto debugGetter = [this]()->bool { return _input._debug; };
        auto debugSetter = [this](bool value) {
            _input._debug = value;
            saveSettings();
        };
        auto preference = new CheckPreference(KINECT_PLUGIN, "Extra Debugging", debugGetter, debugSetter);
        preferences->addPreference(preference);
    }
}

bool KinectPlugin::isSupported() const {
    bool supported = false;
#ifdef HAVE_KINECT
    supported = initializeDefaultSensor();
#endif
    return supported;
}

bool KinectPlugin::activate() {
    InputPlugin::activate();

    loadSettings();

    if (_enabled) {

        // register with userInputMapper
        auto userInputMapper = DependencyManager::get<controller::UserInputMapper>();
        userInputMapper->registerDevice(_inputDevice);

        return initializeDefaultSensor();
    }
    return false;
}

bool KinectPlugin::isHandController() const { 
    bool sensorAvailable = false;
#ifdef HAVE_KINECT
    if (_kinectSensor) {
        BOOLEAN sensorIsAvailable = FALSE;
        HRESULT hr = _kinectSensor->get_IsAvailable(&sensorIsAvailable);
        sensorAvailable = SUCCEEDED(hr) && (sensorIsAvailable == TRUE);
    }
#endif
    return _enabled && _initialized && sensorAvailable;
}


bool KinectPlugin::initializeDefaultSensor() const {
#ifdef HAVE_KINECT
    if (_initialized) {
        return true;
    }

    // Put in a a 5 sec delay - to get in position for the calibration

    Sleep(5000);

    _input._calibrated  = false;     
    _input._debug = true;
    HRESULT hr;

    hr = GetDefaultKinectSensor(&_kinectSensor);
    if (FAILED(hr)) {
        return false;
    }

    if (_kinectSensor) {
        // Initialize the Kinect and get coordinate mapper and the body reader
        IBodyFrameSource* bodyFrameSource = NULL;

        hr = _kinectSensor->Open();

        if (SUCCEEDED(hr)) {
            hr = _kinectSensor->get_CoordinateMapper(&_coordinateMapper);
        }

        if (SUCCEEDED(hr)) {
            hr = _kinectSensor->get_BodyFrameSource(&bodyFrameSource);
        }

        if (SUCCEEDED(hr)) {
            hr = bodyFrameSource->OpenReader(&_bodyFrameReader);
        }

        SafeRelease(bodyFrameSource);
    }

    if (!_kinectSensor || FAILED(hr)) {
        return false;
    }

    _initialized = true;
    return true;
#else
    return false;
#endif
}

void KinectPlugin::updateBody() {
#ifndef HAVE_KINECT
    return;
#else
    if (!_bodyFrameReader) {
        return;
    }

    IBodyFrame* pBodyFrame = NULL;

    HRESULT hr = _bodyFrameReader->AcquireLatestFrame(&pBodyFrame);

    if (SUCCEEDED(hr)) {
        INT64 nTime = 0;
        hr = pBodyFrame->get_RelativeTime(&nTime);
        IBody* bodies[BODY_COUNT] = {0};
        if (SUCCEEDED(hr)) {
            hr = pBodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies);
        }

        if (SUCCEEDED(hr)) {
            ProcessBody(nTime, BODY_COUNT, bodies);
        }

        for (int i = 0; i < _countof(bodies); ++i) {
            SafeRelease(bodies[i]);
        }
    }

    SafeRelease(pBodyFrame);
#endif
}

#ifdef HAVE_KINECT
void KinectPlugin::ProcessBody(INT64 time, int bodyCount, IBody** bodies) {
    bool foundOneBody = false;
    if (_coordinateMapper) {
        for (int i = 0; i < bodyCount; ++i) {
            if (foundOneBody) {
                break;
            }
            IBody* body = bodies[i];
            if (body) {
                BOOLEAN tracked = false;
                HRESULT hr = body->get_IsTracked(&tracked);

                if (SUCCEEDED(hr) && tracked) {
                    foundOneBody = true;

                    if (_joints.size() != JointType_Count) {
                        _joints.resize(JointType_Count, { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f } });
                    }

                    Joint joints[JointType_Count];
                    JointOrientation jointOrientations[JointType_Count];
                    HandState leftHandState = HandState_Unknown;
                    HandState rightHandState = HandState_Unknown;

                    body->get_HandLeftState(&leftHandState);
                    body->get_HandRightState(&rightHandState);

                    hr = body->GetJoints(_countof(joints), joints);
                    hr = body->GetJointOrientations(_countof(jointOrientations), jointOrientations);

                    if (SUCCEEDED(hr)) {
                        auto jointCount = _countof(joints);
                        if (_input._debug) {
                            qDebug() << __FUNCTION__ << "nBodyCount:" << bodyCount << "body:" << i << "jointCount:" << jointCount;
                        }
                        
                        for (int j = 0; j < jointCount; ++j) {

                            glm::vec3 jointPosition { joints[j].Position.X,
                                                      joints[j].Position.Y,
                                                      joints[j].Position.Z };

                            // This is the rotation in the kinect camera/sensor frame... we adjust that in update...
                            // NOTE: glm::quat(W!!!, x, y, z)... not (x,y,z,w)!!!
                            glm::quat jointOrientation { jointOrientations[j].Orientation.w,
                                                         jointOrientations[j].Orientation.x,
                                                         jointOrientations[j].Orientation.y,
                                                         jointOrientations[j].Orientation.z };

                            if (_input._debug) {
                                QString jointName = kinectJointNames[joints[j].JointType];
                                qDebug() << __FUNCTION__ << "joint[" << j << "]:" << jointName
                                        << "position:" << jointPosition
                                        << "orientation:" << jointOrientation
                                        << "isTracked:" << (joints[j].TrackingState != TrackingState_NotTracked);
                            }

                            // filling in the _joints data...
                            if (joints[j].TrackingState != TrackingState_NotTracked) {
                                _joints[j].position = jointPosition;

                                // Kinect Documentation...
                                //
                                // https://social.msdn.microsoft.com/Forums/en-US/31c9aff6-7dab-433d-9af9-59942dfd3d69/kinect-v20-preview-sdk-jointorientation-vs-boneorientation?forum=kinectv2sdk
                                // seems to suggest these are absolute...
                                //    "These quaternions are absolute, so you can take a mesh in local space, transform it by the quaternion, 
                                //    and it will match the exact orientation of the bone.  If you want relative orientation quaternion, you 
                                //    can multiply the absolute quaternion by the inverse of the parent joint's quaternion."
                                //
                                // This is consistent with our findings, but does not include "enough information"
                                //  - Bone direction(Y green) - always matches the skeleton.
                                //  - Normal(Z blue) - joint roll, perpendicular to the bone
                                //  - Binormal(X orange) - perpendicular to the bone and normal

                                // NOTE: Common notation of vectors on paper...
                                //     (+) is the back of the arrow - this vector is pointing into the page
                                //     (o) is the point of the arrow - this vector is pointing out of the page
                                //

                                // From ABOVE the kinect coordinate frame looks like this:
                                //
                                //   Assuming standing facing the kinect camera
                                //   Right Hand with fingers pointing up (green/y)
                                //                   thumb pointing behind body (blue/z)
                                //                   palm facing the head (point out back of my hand, red/x)
                                //
                                //   The identity rotation relative to the cameras frame... (the joint data from SDK)
                                //
                                //              y        | | | |
                                //              |        | | | |
                                //              |        |     |
                                //        z----(o)     \ |right|
                                //                x     \_     |
                                //                        |   |
                                //                        |   |
                                //  
                                //   Expected... identity rotation for left hand..... [to be verified]
                                //   Left Hand with fingers pointing up (green/y)
                                //                   thumb pointing forward (blue/z)
                                //                   palm facing outward away from head (point out back of my hand, red/x)
                                //
                                // Our desired coordinate system... 
                                //     "the local coordinate of the palm in our system"...
                                //
                                // From ABOVE the hand canonical axes look like this:
                                //
                                //
                                //      | | | |          y        | | | |
                                //      | | | |          |        | | | |
                                //      |     |          |        |     |
                                //      |left | /  x----(+)     \ |right|
                                //      |     _/           z     \_     |
                                //       |   |                     |   |
                                //       |   |                     |   |
                                //
                                // Right hand rule... make the hitch hiking sign...
                                //     thumb points in direction of the axis you want to rotate around
                                //     fisted fingers curl in positive rotation direction....
                                //
                                // To transform from Kinect to our RIGHT Hand.... Negative 90 deg around Y
                                // 
                                // FIXME -- Double check if JointType_HandRight vs JointType_WristRight is actually 
                                //          the joint we want to be using!!
                                // 
                                //_joints[j].orientation = jointOrientation;
                                if (joints[j].JointType == JointType_HandRight) {
                                    static const quat kinectToHandRight = glm::angleAxis(-PI / 2.0f, Vectors::UNIT_Y);
                                    // add moving average of orientation quaternion 
                                    glm::quat jointSample = jointOrientation * kinectToHandRight;
                                    if (glm::dot(jointSample, _RightHandOrientationAverage.getAverage()) < 0) {
                                        jointSample = -jointSample;
                                    }
                                    _RightHandOrientationAverage.addSample(jointSample);
                                    _joints[j].orientation = glm::normalize(_RightHandOrientationAverage.getAverage());
                                }  else if (joints[j].JointType == JointType_HandLeft) {
                                    // To transform from Kinect to our LEFT  Hand.... Postive 90 deg around Y
                                    static const quat kinectToHandLeft = glm::angleAxis(PI / 2.0f, Vectors::UNIT_Y);
                                    // add moving average of orientation quaternion 
                                    glm::quat jointSample = jointOrientation * kinectToHandLeft;
                                    if (glm::dot(jointSample, _LeftHandOrientationAverage.getAverage()) < 0) {
                                        jointSample = -jointSample;
                                    }
                                    _LeftHandOrientationAverage.addSample(jointSample);
                                    _joints[j].orientation = glm::normalize(_LeftHandOrientationAverage.getAverage());
                                } else {
                                    _joints[j].orientation = jointOrientation;
                                }

                            }
                        }
                    }
                }
            }
        }
    }
}
#endif


void KinectPlugin::deactivate() {
    // unregister from userInputMapper
    if (_inputDevice->_deviceID != controller::Input::INVALID_DEVICE) {
        auto userInputMapper = DependencyManager::get<controller::UserInputMapper>();
        userInputMapper->removeDevice(_inputDevice->_deviceID);
    }

    InputPlugin::deactivate();
    saveSettings();

#ifdef HAVE_KINECT
    // done with body frame reader
    SafeRelease(_bodyFrameReader);

    // done with coordinate mapper
    SafeRelease(_coordinateMapper);

    // close the Kinect Sensor
    if (_kinectSensor) {
        _kinectSensor->Close();
    }

    SafeRelease(_kinectSensor);
#endif // HAVE_KINECT

}

void KinectPlugin::pluginUpdate(float deltaTime, const controller::InputCalibrationData& inputCalibrationData) {

    if (!_enabled) {
        return;
    }

    updateBody(); // updates _joints

    std::vector<KinectJoint> joints = _joints;

    auto userInputMapper = DependencyManager::get<controller::UserInputMapper>();
    userInputMapper->withLock([&, this]() {
        _inputDevice->update(deltaTime, inputCalibrationData, joints, _prevJoints);
    });

    _prevJoints = joints;
}

void KinectPlugin::saveSettings() const {
    Settings settings;
    QString idString = getID();
    settings.beginGroup(idString);
    {
        settings.setValue(QString("enabled"), _enabled);
        settings.setValue(QString("extraDebug"), _input._debug);
    }
    settings.endGroup();
}

void KinectPlugin::loadSettings() {
    Settings settings;
    QString idString = getID();
    settings.beginGroup(idString);
    {
        _enabled = settings.value("enabled", QVariant(DEFAULT_ENABLED)).toBool();
        _input._debug = settings.value("extraDebug", QVariant(DEFAULT_ENABLED)).toBool();
    }
    settings.endGroup();
}

//
// InputDevice
//

// FIXME - we probably should only return the inputs we ACTUALLY have
controller::Input::NamedVector KinectPlugin::InputDevice::getAvailableInputs() const {
    static controller::Input::NamedVector availableInputs;
    if (availableInputs.size() == 0) {
        for (int i = 0; i < KinectJointIndex::Size; i++) {
            auto channel = KinectJointIndexToPoseIndex(static_cast<KinectJointIndex>(i));
            availableInputs.push_back(makePair(channel, getControllerJointName(channel)));
        }
    };
    return availableInputs;
}

QString KinectPlugin::InputDevice::getDefaultMappingConfig() const {
    static const QString MAPPING_JSON = PathUtils::resourcesPath() + "/controllers/kinect.json";
    return MAPPING_JSON;
}

void KinectPlugin::InputDevice::update(float deltaTime, const controller::InputCalibrationData& inputCalibrationData, 
                                        const std::vector<KinectPlugin::KinectJoint>& joints, const std::vector<KinectPlugin::KinectJoint>& prevJoints) {



    for (size_t i = 0; i < joints.size(); i++) {
       
        QTime currTime = QTime::currentTime();
        QString jointName = kinectJointNames[(JointType)i];
       
        qDebug() << currTime.toString("hh:mm:ss.zzz")
            << "joint[" << i << "]:" << jointName
            << " position: " << joints[i].position
            << "orientation: " << joints[i].orientation;


        if (!_calibrated){

            qDebug() << "_calibrated = " << _calibrated;
            
            if (_avg_joints.size() == 0){
                buildAverageJoints();
            }
                
            averageJoints(joints, i);
           
            if (_avg_joints[i].positionAvg.numSamples >= 300){

                // debug output joint averages
                
                qDebug() << " Joint Averages";

                for (int j = 0; j < JointType_Count; j++){
                    QString jointName = kinectJointNames[(JointType)j];
                    qDebug() << __FUNCTION__ << "joint[" << j << "]:" << jointName
                    << " average position: " << _avg_joints[j].positionAvg.getAverage()
                    << "average orientation: " << _avg_joints[j].orientationAvg.getAverage();
                }
                
                calcCalibrationTargets();
                calculateCalibration();
                TestCalibration();
                deleteAverageJoints();
                _calibrated = true;

            }
            
            if (_calibrated){
                applyTransform(i, deltaTime, joints, prevJoints, inputCalibrationData);
            }
            

        }
    }
}


void KinectPlugin::InputDevice::averageJoints(const std::vector<KinectPlugin::KinectJoint>& joints, const size_t &i){
     
    std::unique_lock<std::mutex> lock(_lock);
    
    _avg_joints[i].positionAvg.addSample(joints[i].position);
 
    glm::quat tmpJointOrientation = joints[i].orientation;
    
    if (glm::dot(joints[i].orientation, _avg_joints[i].orientationAvg.getAverage()) < 0) {
        tmpJointOrientation = -tmpJointOrientation;
    }
    
    tmpJointOrientation = glm::normalize(tmpJointOrientation); // use on instance of ThreadSafeMovingAverage<glm::quat, 2>
    _avg_joints[i].orientationAvg.addSample(tmpJointOrientation);

}


void KinectPlugin::InputDevice::calcCalibrationTargets(){

    std::unique_lock<std::mutex> lock(_lock);
    if (_cal_targets.size() < JointType_Count){
        _cal_targets.resize(JointType_Count);
    }

    for (int i = 0; i < JointType_Count; i++){
        _cal_targets[i].position = { 0.0, 0.0, 0.0, 0.0 };
        _cal_targets[i].orientation = { 0.0, 0.0, 0.0, 0.0 };
    }

    // put targets in from the T Pose here

    for (int i = 0; i < JointType_Count; i++){

        if ((JointType)i == JointType_HandRight){
            _cal_targets[i].position = {0.0f, -0.8f, 0.4f, 0.0f };
            _cal_targets[i].orientation = { 0.5f, 0.5f, -0.5f, 0.5f };
        }

        if ((JointType)i == JointType_HandLeft){
            _cal_targets[i].position = { 0.0f, 0.8f, 0.4f, 0.0f };
            _cal_targets[i].orientation = { 0.5f, 0.5f, 0.5f, -0.5f };
        }

        if ((JointType)i == JointType_FootRight){
            _cal_targets[i].position = { 0.0f, -0.1f, -1.0f, 0.0f };
            _cal_targets[i].orientation = { -4.03841e-08f ,- 1.67276e-08f, -0.382683f, -0.92388f };
        }

        if ((JointType)i == JointType_FootLeft){
            _cal_targets[i].position = { 0.0f, 0.1f, -1.0f, 0.0f };
            _cal_targets[i].orientation = { -4.03841e-08f ,- 1.67276e-08f, 0.382683f, 0.92388f };
        }
 
        if ((JointType)i == JointType_SpineBase){
            _cal_targets[i].position = { 0.0f, 0.0f, 0.0f, 0.0f };
            _cal_targets[i].orientation = { 1.0f, 0.0f, 0.0f, 0.0f };

        }

    }

}



void KinectPlugin::InputDevice::calculateTransforms(const int &i){
    
    float eps = 0.001f;

    // Caculate position transform

        glm::quat q1 = _cal_targets[i].position;
        glm::vec3 v1 = _avg_joints[i].positionAvg.getAverage();
        glm::quat q2(0.0, v1.x, v1.y, v1.z);
        if (glm::abs(glm::dot(q2, q2)) < eps){
            q2 = { 0.0, eps, eps, eps };
        }
        glm::quat q2inv = glm::inverse(q2);
        _cal_trans[i].position = glm::cross(q2inv, q1);

        // calculate orientation transform

        q1 = _cal_targets[i].orientation;
        q2 = _avg_joints[i].orientationAvg.getAverage();
        if (glm::abs(glm::dot(q2, q2)) < eps){
            q2 = { 0.0, eps, eps, eps };
        }
        q2inv = glm::inverse(q2);
        _cal_trans[i].orientation = glm::cross(q2inv, q1);
}

void KinectPlugin::InputDevice::calculateCalibration(){

    if (_cal_trans.size() < JointType_Count){
        _cal_trans.resize(JointType_Count);
    }

    for (int i = 0; i < JointType_Count; i++){
        calculateTransforms(i);
    }

}

void KinectPlugin::InputDevice::buildAverageJoints(){
    std::unique_lock<std::mutex> lock(_lock);
    _avg_joints.resize(JointType_Count);
}

void KinectPlugin::InputDevice::deleteAverageJoints(){
    std::unique_lock<std::mutex> lock(_lock);
    _avg_joints.clear();
}

void KinectPlugin::InputDevice::TestTPose(size_t i){

    KinectJoint tmpJoint;
    int poseIndex = KinectJointIndexToPoseIndex((KinectJointIndex)i);
    glm::vec3 linearVel = { 0.0, 0.0, 0.0 };
    glm::vec3 angularVel = { 0.0, 0.0, 0.0 };

    if ((JointType)i == JointType_HandRight){
        tmpJoint.position = { -0.8, 0.4, 0.0 };
        tmpJoint.orientation = { 0.5f, 0.5f, -0.5f, 0.5f };
        //tmpJoint.orientation = { 1.0, 0.0, 0.0, 0.0 };
        //tmpJoint.orientation = tmpJoint.orientation * glm::angleAxis(PI/2, Vectors::UNIT_NEG_Y);
        //tmpJoint.orientation = tmpJoint.orientation * glm::angleAxis(PI / 2, Vectors::UNIT_X);
        qDebug() << "Right Hand Joint index = " << i;
        qDebug() << "Right Hand pose index = " << poseIndex;
        qDebug() << "Orientation = " << tmpJoint.orientation;
    } else if ((JointType)i == JointType_HandLeft){
        tmpJoint.position = { 0.8, 0.4, 0.0 };
        tmpJoint.orientation = { 0.5f, 0.5f, 0.5f, -0.5f };
        //tmpJoint.orientation = { 1.0, 0.0, 0.0, 0.0 };
        //tmpJoint.orientation = tmpJoint.orientation * glm::angleAxis(PI / 2, Vectors::UNIT_Y);
        //tmpJoint.orientation = tmpJoint.orientation * glm::angleAxis(PI / 2, Vectors::UNIT_X);
        qDebug() << "Left Hand Joint index = " << i;
        qDebug() << "Left Hand pose index = " << poseIndex;
        qDebug() << "Orientation = " << tmpJoint.orientation;
    } else if ((JointType)i == JointType_FootRight){
        tmpJoint.position = { -0.1, -1.0, 0.0 };
        tmpJoint.orientation = { -4.03841e-08f, -1.67276e-08f, -0.382683f, -0.92388f };
        //tmpJoint.orientation = { 1.0, 0.0, 0.0, 0.0 };
        //tmpJoint.orientation = tmpJoint.orientation * glm::angleAxis(PI, Vectors::UNIT_NEG_Z);
        //tmpJoint.orientation = tmpJoint.orientation * glm::angleAxis(PI/4, Vectors::UNIT_X);
        qDebug() << "Right Foot Joint index = " << i;
        qDebug() << "Right Foot pose index = " << poseIndex;
        qDebug() << "Orientation = " << tmpJoint.orientation;
    } else if ((JointType)i == JointType_FootLeft){
        tmpJoint.position = { 0.1, -1.0, 0.0 };
        tmpJoint.orientation = { -4.03841e-08f, -1.67276e-08f, 0.382683f, 0.92388f };
        //tmpJoint.orientation = { 1.0, 0.0, 0.0, 0.0 };
        //tmpJoint.orientation = tmpJoint.orientation * glm::angleAxis(PI, Vectors::UNIT_Z);
        //tmpJoint.orientation = tmpJoint.orientation * glm::angleAxis(PI/4, Vectors::UNIT_X);
        qDebug() << "Left Foot Joint index = " << i;
        qDebug() << "Left Foot pose index = " << poseIndex;
        qDebug() << "Orientation = " << tmpJoint.orientation;
    } else if ((JointType)i == JointType_SpineBase){
        tmpJoint.position = { 0.0, 0.0, 0.0 };
        tmpJoint.orientation = { 1.0, 0.0, 0.0, 0.0 };
        //tmpJoint.orientation = tmpJoint.orientation * glm::angleAxis(PI, Vectors::UNIT_X); // rotate by 180 degrees
        //poseIndex = 20; // do this as an experiment
        qDebug() << "Spine Base Joint index = " << i;
        qDebug() << "Spine Base index = " << poseIndex;
    }
    else {
        return;
    }

    qDebug() << "poseIndex = " << poseIndex << "tmpJoint.position = " << tmpJoint.position << "tmpJoint.orientation = " << tmpJoint.orientation;
    _poseStateMap[poseIndex] = controller::Pose(tmpJoint.position,tmpJoint.orientation, linearVel, angularVel);

}


void  KinectPlugin::InputDevice::TestCalibration(){

    qDebug() << "Test Calibration";

    for (int i = 0; i < JointType_Count; i++){
        glm::vec3 v1 = _avg_joints[i].positionAvg.getAverage();
        glm::quat q2(0.0, v1.x, v1.y, v1.z);
        glm::quat q3 = _cal_trans[i].position;

        glm::quat q4 = _avg_joints[i].orientationAvg.getAverage();
        glm::quat q5 = _cal_trans[i].orientation;

        QString jointName = kinectJointNames[(JointType)i];
        qDebug() << __FUNCTION__ << "joint[" << i << "]:" << jointName
            << " test target position: " << glm::cross(q2,q3)
            << " test target orientation: " << glm::cross(q4,q5);
    }


}

void  KinectPlugin::InputDevice::applyTransform(const size_t &i, float deltaTime, const std::vector<KinectPlugin::KinectJoint>& joints,
    const std::vector<KinectPlugin::KinectJoint>& prevJoints, const controller::InputCalibrationData& inputCalibrationData){


    if (InJointSet(i)){

        int poseIndex = KinectJointIndexToPoseIndex((KinectJointIndex)i);

        // transform position

        glm::vec3 pos = applyPos(i, joints);

        // transform orientation 

        glm::quat rot = applyRot(i, joints);

        // convert to avatar coordinates

        glm::mat4 controllerToAvatar = glm::inverse(inputCalibrationData.avatarMat) * inputCalibrationData.sensorToWorldMat;
        glm::quat controllerToAvatarRotation = glmExtractRotation(controllerToAvatar);

        const glm::vec3 pos_offset(0.0, 0.0, 1.5);              // set a room offset to a z of 1.5 m to allow forward and backward motion
        pos = controllerToAvatarRotation * (pos - pos_offset);            // leave in sensor coordinate space

        rot = controllerToAvatarRotation * rot;

        glm::vec3 linearVel = { 0.0f, 0.0f, 0.0f };
        glm::vec3 angularVel = { 0.0f, 0.0f, 0.0f };

        if (i < prevJoints.size()){

            // transform previous position

            glm::vec3 prevPos = applyPos(i, prevJoints);

            // transform previous orientation

            glm::quat prevRot = applyRot(i, prevJoints);

            // calculate linear and angular velocity

            linearVel = (pos - prevPos) / deltaTime;  // m/s       // sensor space is in meters

            // quat log imaginary part points along the axis of rotation, with length of one half the angle of rotation.
            glm::quat d = glm::log(rot * glm::inverse(prevRot));
            angularVel = glm::vec3(d.x, d.y, d.z) / (0.5f * deltaTime); // radians/s  
        }
        _poseStateMap[poseIndex] = controller::Pose(pos, rot, linearVel, angularVel);
    }
}

bool  KinectPlugin::InputDevice::InJointSet(const size_t &i){
    
    bool ret = false;
   
    ret = ((JointType)i == JointType_HandRight);
    if (ret) {
        return ret;
    }

    ret = ((JointType)i == JointType_HandRight);
    if (ret) {
        return ret;
    }

    ret = ((JointType)i == JointType_HandLeft);
    if (ret) {
        return ret;
    }

    ret = ((JointType)i == JointType_FootRight);
    if (ret) {
        return ret;
    }


    ret = ((JointType)i == JointType_FootLeft);
    if (ret) {
        return ret;
    }
  
    ret = ((JointType)i == JointType_SpineBase);
    if (ret) {
        return ret;
    }

    return ret;
}
const glm::vec3  KinectPlugin::InputDevice::applyPos(const size_t &i, const std::vector<KinectPlugin::KinectJoint>& joints){
    
    glm::vec3 ret = { 0.0, 0.0, 0.0 };

    if (i < joints.size()){

        glm::vec3 v1 = joints[i].position;
        glm::quat q2(0.0, v1.x, v1.y, v1.z);
        glm::quat q3 = _cal_trans[i].position;
        glm::quat qP = glm::cross(q2, q3);

        ret.x = qP.x;
        ret.y = qP.y;
        ret.z = qP.z;
    }

    return ret;
}


const glm::quat  KinectPlugin::InputDevice::applyRot(const size_t &i, const std::vector<KinectPlugin::KinectJoint>& joints){

    glm::quat ret = { 0.0, 0.0, 0.0, 0.0 };

    if (i < joints.size()){

        glm::quat q4 = joints[i].orientation;
        glm::quat q5 = _cal_trans[i].orientation;
        glm::quat qO = glm::cross(q4, q5);
        ret = qO;
    }

    return ret;
}


void KinectPlugin::InputDevice::clearState() {
    for (size_t i = 0; i < KinectJointIndex::Size; i++) {
        int poseIndex = KinectJointIndexToPoseIndex((KinectJointIndex)i);
        _poseStateMap[poseIndex] = controller::Pose();
    }
}