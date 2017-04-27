var handlerId = 0;


var ikTypes = {
    RotationAndPosition: 0,
    RotationOnly: 1,
    HmdHead: 2,
    HipsRelativeRotationAndPosition: 3,
    Off: 4
};

var MAPPING_NAME = "com.highfidelity.examples.kinectToAnimation";
var mapping = Controller.newMapping(MAPPING_NAME);

var recentLeftHand;
var recentRightHand;
var recentLeftFoot;
var recentRightFoot;
var recentHips;


mapping.from(Controller.Hardware.Kinect.LeftHand).debug(true).to(function(pose) { recentLeftHand = pose; });
mapping.from(Controller.Hardware.Kinect.RightHand).debug(true).to(function(pose) { recentRightHand = pose; });
mapping.from(Controller.Hardware.Kinect.LeftFoot).debug(true).to(function(pose) { recentLeftFoot = pose; });
mapping.from(Controller.Hardware.Kinect.RightFoot).debug(true).to(function(pose) { recentRightFoot = pose; });
mapping.from(Controller.Hardware.Kinect.Hips).debug(true).to(function(pose) { recentHips = pose; });



function init() {
    var t = 0;
    var propList = [
        "leftHandType", "leftHandPosition", "leftHandRotation", "rightHandType", "rightHandPosition", "rightHandRotation",
        "leftFootType", "leftFootPosition", "leftFootRotation", "rightFootType", "rightFootPosition", "rightFootRotation",
        "hipsType", "hipsPosition", "hipsRotation"
    ];
    handlerId = MyAvatar.addAnimationStateHandler(function (props) {

        
       var d = new Date();
       var n = d.getMilliseconds();
       var msg =  n.toString();
       var msg1 = " recentHips.translation:";
       var msg = msg.concat(msg1);
       
       Vec3.print(msg,recentHips.translation);
       
       msg2 = n.toString();
       msg3 = " recentHips.rotation";
       msg2 = msg2.concat(msg3);
       
       Vec3.print(msg2,recentHips.rotation);
        //Vec3.print("recentLeftHand.translation:", recentLeftHand.translation);
        //Vec3.print("recentRightFoot.translation:", recentRightFoot.translation);
        //Vec3.print("recentLeftFoot.translation:", recentLeftFoot.translation);

        
       
       
        
        
        //Vec3.print("kinectToAnimation: ",n," recentHips.translation = ",recentHips.translation );
        //Vec3.print("kinectToAnimation: recentHips.rotation = ",recentHips.rotation );
        return {

            rightHandType: ikTypes["RotationAndPosition"],
            rightHandPosition: recentRightHand.translation,
            rightHandRotation: recentRightHand.rotation,
            leftHandType: ikTypes["RotationAndPosition"],
            leftHandPosition: recentLeftHand.translation,
            leftHandRotation: recentLeftHand.rotation,

            rightFootType: ikTypes["RotationAndPosition"],
            rightFootPosition: recentRightFoot.translation,
            rightFootRotation: recentRightFoot.rotation,
            leftFootType: ikTypes["RotationAndPosition"],
            leftFootPosition: recentLeftFoot.translation,
            leftFootRotation: recentLeftFoot.rotation,
            hipsType:   ikTypes["RotationAndPosition"], 
            hipsPosition: recentHips.translation, 
            hipsRotation: recentHips.rotation
        };
    }, propList);

    Controller.enableMapping(MAPPING_NAME);
}

init();
Script.scriptEnding.connect(function(){
    MyAvatar.removeAnimationStateHandler(handlerId);
    mapping.disable();
});



