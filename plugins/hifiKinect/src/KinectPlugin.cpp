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
#include <glm/gtx/string_cast.hpp>

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

const char* controllerJointNames[] = {
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
        return controllerJointNames[i];
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

    Sleep(5);

    _input._calibrated  = false;     
    _input._debug = false;
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

                 if (SUCCEEDED(hr) && tracked ) {
                    foundOneBody = true;

                    if (_input._joints.size() != JointType_Count) {
                        _input._joints.resize(JointType_Count, { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f } });
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
                        _input._debug = true;
                        if (_input._debug) {
                          //  qDebug() << __FUNCTION__ << "nBodyCount:" << bodyCount << "body:" << i << "jointCount:" << jointCount;
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

                            //if (_input._debug) {
                                //_input.printJoint(joints[j],joints[j].JointType,jointPosition,jointOrientation);
                            //}

                            _input._debug = false;
                            // filling in the _input._joints data...
                            if (joints[j].TrackingState != TrackingState_NotTracked) {
                                _input._joints[j].position = jointPosition;

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
                                    _input._joints[j].orientation = glm::normalize(_RightHandOrientationAverage.getAverage());
                                }  else if (joints[j].JointType == JointType_HandLeft) {
                                    // To transform from Kinect to our LEFT  Hand.... Postive 90 deg around Y
                                    static const quat kinectToHandLeft = glm::angleAxis(PI / 2.0f, Vectors::UNIT_Y);
                                    // add moving average of orientation quaternion 
                                    glm::quat jointSample = jointOrientation * kinectToHandLeft;
                                    if (glm::dot(jointSample, _LeftHandOrientationAverage.getAverage()) < 0) {
                                        jointSample = -jointSample;
                                    }
                                    _LeftHandOrientationAverage.addSample(jointSample);
                                    _input._joints[j].orientation = glm::normalize(_LeftHandOrientationAverage.getAverage());
                                } else {
                                    _input._joints[j].orientation = jointOrientation;
                                }
                                //_input.printJoint(_input._joints[j],(JointType)j);
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

    if (_input._joints.size() != JointType_Count) {
        _input._joints.resize(JointType_Count, { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f } });
    }

    updateBody(); // updates _joints

    
    std::vector<KinectJoint> joints = _input._joints;

    auto userInputMapper = DependencyManager::get<controller::UserInputMapper>();
    //userInputMapper->withLock([&, this]() {
        _inputDevice->update(deltaTime, inputCalibrationData, joints, _input._prevJoints);
    //});

    _input._prevJoints = joints;
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
            //qDebug() << "AJT: makePair" << channel << "->" << getControllerJointName(channel);
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


// if (_debug){
        //     printJoint(joints[i], (JointType)i);
        // }applyT
        if (!_calibrated){

            //qDebug() << "_calibrated = " << _calibrated;

            if (_avg_joints.size() == 0){
                buildAverageJoints();
            }

            KinectJoint tmpJoint = joints[i];

            //KinectJoint tmpJoint = TestDefaultPose(i,inputCalibrationData);
            
            if (_joints.size() != JointType_Count) {
            _joints.resize(JointType_Count);
            } 
            _joints[i] = tmpJoint;


            bool flag = InJointSet(i);
            if (flag) {
                averageJoints(tmpJoint, i);   // changed definition to use single joint
            }

            if (_avg_joints[i].positionAvg.numSamples >= 500){

                printJointAvg();

                buildCalVector();

                calibrate(inputCalibrationData);

                //qDebug() << "Test Calibration";

                //TestCalibration();
                _calibrated = true;
            }
        }

        if (_calibrated){

            if (InJointSet(i)){

                //KinectJoint test = TestDefaultPose(i,inputCalibrationData);

                /*if((JointType)i == JointType_HandRight){
                    float angle = PI / 6;
                    glm::quat qtmp = { cos(angle), 0.0, 0.0, sin(angle) }; // PI/3 around z axis
                    glm::normalize(qtmp);
                    test.position = testRotation(qtmp, test.position); // rotate right hand PI/3 around z - axis
                    qDebug() << "Test Position = " << test.position;
                    } */

                KinectJoint test = joints[i];
               
               /* glm::mat4 sensorToAvatarMat = glm::inverse(inputCalibrationData.avatarMat) * inputCalibrationData.sensorToWorldMat;
                glm::mat4 jointMat = createMatFromQuatAndPos(test.orientation, test.position);
                jointMat = sensorToAvatarMat*jointMat;

                test.position = extractTranslation(jointMat);
                test.orientation = glmExtractRotation(jointMat); */

                glm::vec3 linearVel = { 0.0, 0.0, 0.0 };
                glm::vec3 angularVel = { 0.0, 0.0, 0.0 };

                applyTransform(i, deltaTime, test, prevJoints[i], inputCalibrationData);

                 //int poseIndex = KinectJointIndexToPoseIndex((KinectJointIndex)i);
                 //_poseStateMap[poseIndex] = controller::Pose(test.position, test.orientation, linearVel, angularVel);
                 //printPoseStateMap(i); 
            }
        }
    }
}


void KinectPlugin::InputDevice::calibrate(const controller::InputCalibrationData &inputCalibration){



    QString defaultHeadMat = QString::fromStdString(glm::to_string(inputCalibration.defaultHeadMat));
    qDebug() << "InputCalibrationData = " 
        << "defaultHeadMat = " << defaultHeadMat
        << "avatarMat = " << inputCalibration.avatarMat
        << "default hips = " << inputCalibration.defaultHips
        << "left foot = " << inputCalibration.defaultLeftFoot
        << " right foot = " << inputCalibration.defaultRightFoot; 
   
    // convert the hmd head from sensor space to avatar space

    glm::vec3 pos = _avg_joints[JointType_Head].positionAvg.getAverage();
    glm::quat rot = _avg_joints[JointType_Head].orientationAvg.getAverage();

    // Test print to vec3 and quat 

    QString Qpos = QString::fromStdString(glm::to_string(pos));
    QString Qrot = QString::fromStdString(glm::to_string(rot));

    qDebug() << "Qpos = " << Qpos;
    qDebug() << "Qrot = " << Qrot;

    glm::mat4 head = createMatFromQuatAndPos(rot, pos);

    glm::mat4 hmdSensorFlippedMat = head * Matrices::Y_180;
    glm::mat4 sensorToAvatarMat = glm::inverse(inputCalibration.avatarMat) * inputCalibration.sensorToWorldMat;
    glm::mat4 hmdAvatarMat = sensorToAvatarMat * hmdSensorFlippedMat;

    // cancel the roll and pitch for the hmd head
    glm::quat hmdRotation = cancelOutRollAndPitch(glmExtractRotation(hmdAvatarMat));
    glm::vec3 hmdTranslation = extractTranslation(hmdAvatarMat);
    glm::mat4 currentHmd = createMatFromQuatAndPos(hmdRotation, hmdTranslation);

    // calculate the offset from the centerOfEye to defaultHeadMat
    glm::mat4 defaultHeadOffset = glm::inverse(inputCalibration.defaultCenterEyeMat) * inputCalibration.defaultHeadMat;

    glm::mat4 currentHead = currentHmd * defaultHeadOffset;

    // calculate the defaultToRefrenceXform
    glm::mat4 defaultToReferenceMat = currentHead * glm::inverse(inputCalibration.defaultHeadMat);

    // Create transform to move default avatar pose to under the head and calulate joint off 

    pos = _avg_joints[JointType_FootLeft].positionAvg.getAverage();
    rot = _avg_joints[JointType_FootLeft].orientationAvg.getAverage();

    _cal_trans[JointType_FootLeft] = computeOffset(defaultToReferenceMat, inputCalibration.defaultLeftFoot, sensorToAvatarMat, pos, rot);

    pos = _avg_joints[JointType_FootRight].positionAvg.getAverage();
    rot = _avg_joints[JointType_FootRight].orientationAvg.getAverage();

    _cal_trans[JointType_FootRight] = computeOffset(defaultToReferenceMat, inputCalibration.defaultRightFoot, sensorToAvatarMat, pos, rot);

    pos = _avg_joints[JointType_HandLeft].positionAvg.getAverage();
    rot = _avg_joints[JointType_HandLeft].orientationAvg.getAverage();

    _cal_trans[JointType_HandLeft] = computeOffset(defaultToReferenceMat, inputCalibration.defaultSpine2, sensorToAvatarMat, pos, rot);

    pos = _avg_joints[JointType_HandRight].positionAvg.getAverage();
    rot = _avg_joints[JointType_HandRight].orientationAvg.getAverage();

    _cal_trans[JointType_HandRight] = computeOffset(defaultToReferenceMat, inputCalibration.defaultSpine2, sensorToAvatarMat, pos, rot);


    pos = _avg_joints[JointType_SpineBase].positionAvg.getAverage();
    rot = _avg_joints[JointType_SpineBase].orientationAvg.getAverage();

    _cal_trans[JointType_SpineBase] = computeOffset(defaultToReferenceMat, inputCalibration.defaultHips, sensorToAvatarMat, pos, rot);

    pos = _avg_joints[JointType_SpineMid].positionAvg.getAverage();
    rot = _avg_joints[JointType_SpineMid].orientationAvg.getAverage();

    _cal_trans[JointType_SpineMid] = computeOffset(defaultToReferenceMat, inputCalibration.defaultSpine2, sensorToAvatarMat, pos, rot);

}


glm::mat4 KinectPlugin::InputDevice::computeOffset(glm::mat4 defaultToReferenceMat, glm::mat4 defaultJointMat, glm::mat4 sensorToAvatarMat, glm::vec3 jointPos, glm::quat jointRot) {
    glm::mat4 poseMat = createMatFromQuatAndPos(jointRot, jointPos);
    // convert to avatar space 
    poseMat = sensorToAvatarMat*poseMat;
    glm::mat4 referenceJointMat = defaultToReferenceMat * defaultJointMat;
    return glm::inverse(poseMat) * referenceJointMat;
}


void KinectPlugin::InputDevice::averageJoints(const KinectJoint &joint, const size_t &i){
     
    std::unique_lock<std::mutex> lock(_lock);

    //qDebug() << __FUNCTION__ << "Num Samples = " << _avg_joints[i].positionAvg.numSamples;
    
    //qDebug() << __FUNCTION__ << "Input Joint";

    //printJoint(joint, (JointType)i);

    _avg_joints[i].positionAvg.addSample(joint.position);
 
    glm::quat tmpJointOrientation = joint.orientation;
    
//    if (glm::dot(joint.orientation, _avg_joints[i].orientationAvg.getAverage()) < 0) {
//        tmpJointOrientation = -tmpJointOrientation;
 //   }

    tmpJointOrientation = glm::normalize(tmpJointOrientation);
    _avg_joints[i].orientationAvg.addSample(tmpJointOrientation);

    //qDebug() << __FUNCTION__ << "Averaging Joint";

    KinectJoint tmpAvgJoint;

    tmpAvgJoint.position = _avg_joints[i].positionAvg.getAverage();
    tmpAvgJoint.orientation = _avg_joints[i].orientationAvg.getAverage();

    //printJoint(tmpAvgJoint, (JointType)i);
}


void KinectPlugin::InputDevice::calcCalibrationTargets(){

    std::unique_lock<std::mutex> lock(_lock);
    if (_cal_targets.size() < JointType_Count){
        _cal_targets.resize(JointType_Count);
    }

    for (int i = 0; i < JointType_Count; i++){
        _cal_targets[i].position = { 0.0, 0.0, 0.0 };
        _cal_targets[i].orientation = { 0.0, 0.0, 0.0, 0.0 };
    }

    // put targets in from the T Pose here
    glm::vec3 hips = _localBasis.hips;

    for (int i = 0; i < JointType_Count; i++){

        if ((JointType)i == JointType_HandRight){
            _cal_targets[i].position = { 0.8f, 0.4f, 0.0f };
            _cal_targets[i].orientation = { 0.5f, 0.5f, 0.5f, -0.5f };
            //_cal_targets[i].orientation = { , 0.0,1.0f, 0.0f };
        }

        if ((JointType)i == JointType_ElbowRight){
            _cal_targets[i].position = { 0.48f, 0.4f, 0.0f };
            _cal_targets[i].orientation = { 1.0f, 0.0f, 0.0f, 0.0f };
        }

        if ((JointType)i == JointType_HandLeft){
            _cal_targets[i].position = { -0.8f, 0.4f, 0.0f };
            _cal_targets[i].orientation = { 0.5f, 0.5f, -0.5f, 0.5f };
        }

        if ((JointType)i == JointType_FootRight){
            _cal_targets[i].position = { 0.1f, -1.0f, 0.0f };
            _cal_targets[i].orientation = { 0.382683f, -0.92388f, 0.0f, 0.0f };
        }

        if ((JointType)i == JointType_FootLeft){
            _cal_targets[i].position = { -0.1f, -1.0f, 0.0f };
            _cal_targets[i].orientation = { -0.382683f, 0.92388f, 0.0f, 0.0f };
        }

        if ((JointType)i == JointType_SpineBase){
            _cal_targets[i].position = { 0.0f, 0.0f, 0.0f };
            _cal_targets[i].orientation = { 0.0f, 0.0f, 1.0f, 0.0f };
        }

    }
}

void KinectPlugin::InputDevice::calculateTransforms(const size_t &i){
    
    
    glm::vec3  vin = _avg_joints[i].positionAvg.getAverage();
    glm::vec3 vT = _cal_targets[i].position;

    float mag1 = sqrt(glm::dot(vin, vin));
    float mag2 = sqrt(glm::dot(vT, vT));

    if (mag1 != 0 && mag2 != 0){
        // calculate quaternion that rotates vin to vT
        glm::quat pos = rotationBetween(vin, vT);
      //  _cal_trans[i].position = glm::normalize(pos);
    }

    calculateOrientations(i);
}


void  KinectPlugin::InputDevice::calculateOrientations(const size_t &i){

    glm::vec3 vTa = { 0.0, 0.0, 0.0 };
    glm::vec3 vT1a = { 0.0, 0.0, 0.0 };
    glm::quat qC = { 1.0, 0.0, 0.0, 0.0 };
    
    if ((JointType)i == JointType_HandRight){
        vTa = _avg_joints[i].positionAvg.getAverage();
        vT1a = _avg_joints[JointType_ElbowRight].positionAvg.getAverage();
    }
    else {
   //     _cal_trans[i].orientation = qC;
        return;
    }

    glm::quat qT = calculatePoseOrientation(vTa, vT1a);
    glm::quat qA = _avg_joints[i].orientationAvg.getAverage();
    if (glm::dot(qA, qA) != 0.0) {
        qC = glm::inverse(qA)*qT;
    }

//    _cal_trans[i].orientation = qC;
}


glm::quat KinectPlugin::InputDevice::calculatePoseOrientation(const glm::vec3 &vT, const glm::vec3 &vT1){

    glm::vec3 vAxis = vT - vT1;
    vAxis = glm::normalize(vAxis);
    glm::quat qT = { 0.0f, vAxis.x, vAxis.y, vAxis.z };
    if (glm::dot(qT, qT) != 0.0) {
        qT = glm::normalize(qT);
    }
    glm::vec3 vAxis1 = glm::cross(vT, vT1);
    if (glm::dot(vAxis1, vAxis1) != 0.0) {
        vAxis1 = glm::normalize(vAxis1);
    }
    qT = qT * glm::angleAxis(-PI / 2, vAxis);
    qT = qT * glm::angleAxis(-PI / 2, vAxis1);

    return qT;
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


void KinectPlugin::InputDevice::buildCalVector() {

    if (_cal_trans.size() != JointType_Count) {
        _cal_trans.resize(JointType_Count);
    }

    glm::mat4 I(1.0f);  // 4x4 idenity matrix

    for (int i = 0; i < JointType_Count; i++){

        _cal_trans[i] = I;
    }


}


void KinectPlugin::InputDevice::deleteAverageJoints(){
    std::unique_lock<std::mutex> lock(_lock);
    _avg_joints.clear();
}

KinectPlugin::KinectJoint KinectPlugin::InputDevice::TestTPose(size_t i){


    KinectJoint tmpJoint;
    KinectJoint ret;

    int poseIndex = KinectJointIndexToPoseIndex((KinectJointIndex)i);
    glm::vec3 linearVel = { 0.0, 0.0, 0.0 };
    glm::vec3 angularVel = { 0.0, 0.0, 0.0 };

    if ((JointType)i == JointType_HandRight){
        tmpJoint.position = { -0.8f, 0.4f, 0.0f };
        //tmpJoint.orientation = { -0.5f, -0.5f, -0.5f, 0.5f };
        tmpJoint.orientation = { 1.0f, 0.0f, 0.0f, 0.0f };
        
       
    } else if ((JointType)i == JointType_HandLeft){
        tmpJoint.position = { 0.8f, 0.4f, 0.0f };
        tmpJoint.orientation = { -0.5f, -0.5f, 0.5f, -0.5f };
       
       
    } else if ((JointType)i == JointType_FootRight){
        tmpJoint.position = { -0.1f, -1.0f, 0.0f };
        tmpJoint.orientation = { 0.382683f, -0.92388f,0.0f, 0.0f };
       
    } else if ((JointType)i == JointType_FootLeft){
        tmpJoint.position = { 0.1f, -1.0f, 0.0f };
        tmpJoint.orientation = { -0.382683f, 0.92388f, 0.0f, 0.0f };
        
    } else if ((JointType)i == JointType_SpineBase){
        tmpJoint.position = { 0.0f, 0.0f, 0.0f };
        tmpJoint.orientation = { 0.0f, 0.0f,1.0f, 0.0f };
    }
    else {
        ret.position = { 0.0f, 0.0f, 0.0f };
        ret.orientation = { 0.0f, 0.0f, 0.0f, 0.0f };
        return ret;
    }

    glm::vec3 test;


    test = tmpJoint.position * glm::angleAxis(PI, Vectors::UNIT_Y); // rotate by 180 degrees
    tmpJoint.position = test;

    ret.position = tmpJoint.position;
    ret.orientation = tmpJoint.orientation;
    return ret;
}


KinectPlugin::KinectJoint KinectPlugin::InputDevice::TestTPose1(size_t i){

    KinectJoint tmpJoint;
    KinectJoint ret;

    glm::vec3 linearVel = { 0.0, 0.0, 0.0 };

    glm::vec3 angularVel = { 0.0, 0.0, 0.0 };

    if ((JointType)i == JointType_HandRight){
        tmpJoint.position = { 0.8f, 0.4f, 0.0f };
        tmpJoint.orientation = { 0.5f, 0.5f, 0.5f, -0.5f };
    } 
    else if ((JointType)i == JointType_ElbowRight){
        tmpJoint.position = { 0.48f, 0.4f, 0.0f };
        tmpJoint.orientation = { 1.0f, 0.0f, 0.0f, 0.0f };
    }
    else if ((JointType)i == JointType_HandLeft){
        tmpJoint.position = { -0.8f, 0.4f, 0.0f };
        tmpJoint.orientation = { 0.5f, 0.5f, -0.5f, 0.5f };
    }
    else if ((JointType)i == JointType_FootRight){
        tmpJoint.position = { 0.1f, -1.0f, 0.0f };
        tmpJoint.orientation = { 0.382683f, -0.92388f, 0.0f, 0.0f };
    }
    else if ((JointType)i == JointType_FootLeft){
        tmpJoint.position = { -0.1f, -1.0f, 0.0f };
        tmpJoint.orientation = { -0.382683f, 0.92388f, 0.0f, 0.0f };
    } 
    else if ((JointType)i == JointType_SpineBase){
        tmpJoint.position = { 0.0f, 0.0f, 0.0f };
        tmpJoint.orientation = { 0.0f, 0.0f,1.0f, 0.0f };
    }
    else {
        ret.position = { 0.0f, 0.0f, 0.0f };
        ret.orientation = { 0.0f, 0.0f, 0.0f, 0.0f };
        return ret;
    }

    glm::vec3 test;

    ret.position = tmpJoint.position;
    ret.orientation = tmpJoint.orientation;
    return ret;
}

    KinectPlugin::KinectJoint KinectPlugin::InputDevice::TestDefaultPose(const size_t &i, const controller::InputCalibrationData &inputCalibration) {

    glm::mat4 tmp;
    KinectJoint ret;
    
    
    if ((JointType)i == JointType_FootRight){
        tmp = inputCalibration.defaultRightFoot;
        ret.position = extractTranslation(tmp);
        ret.orientation = extractRotation(tmp);
    } else if ((JointType)i == JointType_FootLeft){
        tmp = inputCalibration.defaultLeftFoot;
        ret.position = extractTranslation(tmp);
        ret.orientation = extractRotation(tmp);
    }
    else if ((JointType)i == JointType_SpineMid){
        tmp = inputCalibration.defaultSpine2;
        ret.position = extractTranslation(tmp);
        ret.orientation = extractRotation(tmp);
    } else if ((JointType)i == JointType_SpineBase){
        tmp = inputCalibration.defaultHips;
        ret.position = extractTranslation(tmp);
        ret.orientation = extractRotation(tmp);
        ret.orientation = { 0.0f, 0.0f, 1.0f, 0.0f };
    }
    
    qDebug() << "TestDefaultPose";
    printJoint(ret, (JointType)i);

    return ret;
}




void  KinectPlugin::InputDevice::TestCalibration(){

    for (int i = 0; i < JointType_Count; i++) {
        glm::vec3 v1 = _avg_joints[i].positionAvg.getAverage();
//        glm::quat qPos = _cal_trans[i].position;



        glm::quat q4 = _avg_joints[i].orientationAvg.getAverage();
//        glm::quat q5 = _cal_trans[i].orientation;

        KinectJoint tmpJoint;

       // tmpJoint.position = qPos*v1;
        
       // tmpJoint.orientation = q4*q5;
        
        QString jointName = kinectJointNames[(JointType)i];
        
        qDebug() << "Calibrated Joint";
        printJoint(tmpJoint, (JointType)i);

        qDebug() << "Input Joint";
        tmpJoint = TestTPose1(i);
        printJoint(tmpJoint, (JointType)i);

        qDebug() << "Averaged Joint";
        tmpJoint.position = _avg_joints[i].positionAvg.getAverage();
        tmpJoint.orientation = _avg_joints[i].orientationAvg.getAverage();
        printJoint(tmpJoint, (JointType)i); 

    }

}

void KinectPlugin::InputDevice::applyTransform(const size_t &i, float deltaTime, const KinectJoint &joint,
    const KinectJoint &prevJoints, const controller::InputCalibrationData& inputCalibrationData) {

    
    bool flag = InJointSet(i);
    if (flag){

       qDebug() << "applyTransform";

        int poseIndex = KinectJointIndexToPoseIndex((KinectJointIndex)i);

        glm::vec3 pos = joint.position;
        glm::quat rot = joint.orientation;

        glm::mat4 jointMat = createMatFromQuatAndPos(rot, pos);

        jointMat = _cal_trans[i] * jointMat;

        const glm::vec3 pos_offset(0.0, 0.0, 1.5);              // set a room offset to a z of 1.5 m to allow forward and backward motion
        //glm::vec3 pos_offset = { 0.0, 0.0, 0.0 };
//        pos = controllerToAvatarRotation * (pos - pos_offset);            // leave in sensor coordinate space

        glm::vec3 linearVel = { 0.0f, 0.0f, 0.0f };
        glm::vec3 angularVel = { 0.0f, 0.0f, 0.0f };

        // transform previous position
        
        

        glm::vec3 pPos = prevJoints.position;
        glm::quat pRot = prevJoints.orientation;

        glm::mat4 prevJointMat = createMatFromQuatAndPos(rot, pos);

        prevJointMat = _cal_trans[i]*prevJointMat;
        glm::vec3 prevPos = extractTranslation(prevJointMat);
        glm::quat prevRot = glmExtractRotation(prevJointMat);

        linearVel = (pos - (prevPos * METERS_PER_CENTIMETER)) / deltaTime;  // m/s

        // quat log imaginary part points along the axis of rotation, with length of one half the angle of rotation.
        if (glm::dot(prevRot, prevRot) != 0.0) {
            glm::quat d = glm::log(rot * glm::inverse(prevRot));
            angularVel = glm::vec3(d.x, d.y, d.z) / (0.5f * deltaTime); // radians/s
        }
        
        
        _poseStateMap[poseIndex] = controller::Pose(pos, rot, linearVel, angularVel);
        printPoseStateMap(i);
    }
}

bool  KinectPlugin::InputDevice::InAvJointSet(const size_t &i){
    
    bool ret = false;
   
    ret = ((JointType)i == JointType_Head);
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
  
    ret = ((JointType)i == JointType_SpineMid);
    if (ret) {
        return ret;
    } 

    ret = ((JointType)i == JointType_SpineBase);
    if (ret) {
        return ret;
    }

    return ret;
}


bool  KinectPlugin::InputDevice::InJointSet(const size_t &i){

    bool ret = false;

    ret = ((JointType)i == JointType_Head);
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

    ret = ((JointType)i == JointType_SpineMid);
    if (ret) {
        return ret;
    }

    ret = ((JointType)i == JointType_SpineBase);
    if (ret) {
        return ret;
    }

    return ret;


}


/*const glm::vec3  KinectPlugin::InputDevice::applyPos(const size_t &i, const KinectJoint &joint){
    
    //glm::vec3 v1 = joint.position - _transAdj;
    
    glm::vec3 v1 = joint.position;
//    glm::vec3 ret = _cal_trans[i].position*v1;

    return ret;
} */


/* const glm::quat  KinectPlugin::InputDevice::applyRot(const size_t &i, const glm::quat &rot){

    glm::quat ret = { 0.0, 0.0, 0.0, 0.0 };

    glm::quat q4 = rot;
//    glm::quat q5 = _cal_trans[i].orientation;

//    glm::quat qO = q4*q5;

    ret = qO;

    return ret;
} */


glm::quat  KinectPlugin::InputDevice::alignOrientation(const KinectJoint &joint, const JointType jType){

    // rotate the quaternion to be along the bone

    if (glm::dot(joint.orientation, joint.orientation) == 0.0 || jType == JointType_SpineBase ) {
        return joint.orientation;
    }

    qDebug() << "alignOrientation" << " jType = " << jType;

    glm::vec3 vT;
    glm::vec3 vT1;
    glm::quat ret = { 1.0f, 0.0f, 0.0f, 0.0f };

    if (jType == JointType_HandRight){

        vT = joint.position;
        KinectJoint tmpJoint = _joints[JointType_ElbowRight];
        //vT1 = applyPos(JointType_ElbowRight, tmpJoint);
        //ret = calculatePoseOrientation(vT, vT1);
    }
    else {
        ret = joint.orientation;
    }

    return ret;
}

void KinectPlugin::InputDevice::translationCalibrate(){

    glm::vec3 kinectHip = _avg_joints[0].positionAvg.getAverage();
    glm::vec3 kinectHipcal = { _cal_targets[0].position.x, _cal_targets[0].position.y, _cal_targets[0].position.z };
    glm::vec3 diffPos = kinectHip - kinectHipcal;
    _transAdj = diffPos;

    for (int i = 0; i < JointType_Count; i++){
        _avg_joints[i].positionAvg.average = _avg_joints[i].positionAvg.getAverage() - _transAdj;
    }
}



void KinectPlugin::InputDevice::updateLocalBasis(){
    
    _localBasis.x = _localBasis.x + _localBasis.hips.x;
    _localBasis.y = _localBasis.x + _localBasis.hips.y;
    _localBasis.z = _localBasis.x + _localBasis.hips.z;
}



void KinectPlugin::InputDevice::setLocalBasis(){

    glm::vec3 hips = _avg_joints[0].positionAvg.getAverage();

    _localBasis.x = { 1.0 + hips.x, 0.0, 0.0 };
    _localBasis.y = { 0.0, 1.0 + hips.y, 0.0 };
    _localBasis.z = { 0.0, 0.0, 1.0 + hips.z };
}

glm::vec3 KinectPlugin::InputDevice::transformLocalBasis(glm::vec3 pos){

    glm::vec3 ret;

    ret.x = glm::dot(pos, _localBasis.x);
    ret.y = glm::dot(pos, _localBasis.y);
    ret.z = glm::dot(pos, _localBasis.z);

    return ret;
}

void  KinectPlugin::InputDevice::printJoint(const KinectJoint &joint, const JointType &jointType)
{
    QTime currTime = QTime::currentTime();
    QString jointName = kinectJointNames[jointType];

    qDebug() << currTime.toString("hh:mm:ss.zzz")
         << "joint = " << jointName
        << "position:" << joint.position
        << "orientation:" << joint.orientation;
}

void KinectPlugin::InputDevice::printJoint(const Joint &joint, const JointType &jointType, glm::vec3 jointPosition, glm::quat jointOrientation) {
    
    QTime currTime = QTime::currentTime();
    QString jointName = kinectJointNames[jointType];

    qDebug() << currTime.toString("hh:mm:ss.zzz")
        << __FUNCTION__ << "joint = " << jointName
        << "position:" << jointPosition
        << "orientation:" << jointOrientation;
}

void KinectPlugin::InputDevice::printJointAvg() {

    QTime currTime = QTime::currentTime();

    qDebug() << " Joint Averages";
    qDebug() << currTime.toString("hh:mm:ss.zzz");

    for (int j = 0; j < JointType_Count; j++){
        QString jointName = kinectJointNames[(JointType)j];
        qDebug() << __FUNCTION__ << "joint[" << j << "]:" << jointName
            << " average position: " << _avg_joints[j].positionAvg.getAverage()
            << "average orientation: " << _avg_joints[j].orientationAvg.getAverage();
    }
}

void KinectPlugin::InputDevice::printJointCalTargets(){

    QTime currTime = QTime::currentTime();

    qDebug() << " Joint Calibration Targets";
    qDebug() << currTime.toString("hh:mm:ss.zzz");

    for (int j = 0; j < JointType_Count; j++){

        glm::vec3 pos = { _cal_targets[j].position.x, _cal_targets[j].position.y, _cal_targets[j].position.z };
        glm::quat rot = _cal_targets[j].orientation;
        QString jointName = kinectJointNames[(JointType)j];
        qDebug() << __FUNCTION__ << "joint[" << j << "]:" << jointName
            << " average position: " << pos
            << "average orientation: " << rot;
    }


}


KinectPlugin::KinectJoint KinectPlugin::InputDevice::getJointAverage(const size_t &i) {

    KinectPlugin::KinectJoint ret;

    ret.position = _avg_joints[i].positionAvg.getAverage();
    ret.orientation = _avg_joints[i].orientationAvg.getAverage();

    return ret;
}

void KinectPlugin::InputDevice::printPoseStateMap(const size_t &i){

    QTime currTime = QTime::currentTime();
    size_t N = _poseStateMap.size();
    size_t JointIndex = KinectJointIndex::Size;
     
    if (i == 0){
        qDebug() << " _poseStateMap";
        qDebug() << currTime.toString("hh:mm:ss.zzz");
    }

    int poseIndex = KinectJointIndexToPoseIndex((KinectJointIndex)i);
    glm::vec3 pos = _poseStateMap[poseIndex].translation;
    glm::quat rot = _poseStateMap[poseIndex].rotation;

    qDebug() << "i = " << (long int) ((KinectJointIndex)i)
        << " poseIndex = " << poseIndex
        << " standard joint name = " << controllerJointNames[(size_t)poseIndex]
        << " Kinect Joint Names " << kinectJointNames[i]
        << " position = " << pos
        << " orientation = " << rot;
}


KinectPlugin::KinectJoint KinectPlugin::InputDevice::testTranslation(const KinectJoint &joint, glm::vec3 deltaV){

    KinectJoint ret;

    ret.position = joint.position + deltaV;
    return ret;
}


glm::vec3 KinectPlugin::InputDevice::testRotation(glm::quat q, glm::vec3 v){

    q = glm::normalize(q);
    glm::vec3 ret = q*v;

    return ret;
}


void KinectPlugin::InputDevice::clearState() {
    for (size_t i = 0; i < KinectJointIndex::Size; i++) {
        int poseIndex = KinectJointIndexToPoseIndex((KinectJointIndex)i);
        _poseStateMap[poseIndex] = controller::Pose();
    }
}
