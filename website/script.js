// Parameters - change these to match the Pi
var ip = "131.230.197.82"; 
var port = "9090"; 
var url_string = `ws://${ip}:${port}`;

// Connect to and set up ros bridge
var ros = new ROSLIB.Ros({
    url: url_string
});

ros.on('connection', function() {
    console.log('Connected to ROSBridge!');
});

ros.on('error', function(error) {
    console.log('Error connecting to ROSBridge:', error);
});

ros.on('close', function() {
    console.log('Connection to ROSBridge closed');
    setTimeout(() => ros.connect(url_string), 5000);
});

// Define topics based on the Python nodes
var axisTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/gamepad_axis',
    messageType: 'std_msgs/Int8MultiArray'
});

var buttonTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/gamepad_button', 
    messageType: 'std_msgs/Int8MultiArray'
});

var imageTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/camera/image/compressed',
    messageType: 'sensor_msgs/CompressedImage'
});

// Subscribe to compressed image topic
imageTopic.subscribe(function(message) {
    document.getElementById("video_out").src = "data:image/jpeg;base64," + message.data;
});

// Connect gamepad
window.addEventListener("gamepadconnected", function(e) {
    console.log("Gamepad connected:", e.gamepad);
    setInterval(readControllerData, 75);
});

function readControllerData() {
    var gamepad = navigator.getGamepads()[0];

    if (!gamepad || !ros.isConnected) {
        return;
    }

    // Map axes data - focusing on left stick X/Y as per xbox_translator.py
    var gamepad_axis = new Int8Array([
        parseInt(gamepad.axes[0] * 100), // lstick_x
        parseInt(gamepad.axes[1] * 100)  // lstick_y
    ]);

    // Map button data - focusing on dpad left/right for camera control
    var gamepad_button = new Int8Array(16); // Initialize array with zeros
    gamepad_button[14] = gamepad.buttons[14] ? 100 : 0; // dpad_left
    gamepad_button[15] = gamepad.buttons[15] ? 100 : 0; // dpad_right

    // Publish axis data
    var axisMsg = new ROSLIB.Message({
        data: Array.from(gamepad_axis)
    });
    axisTopic.publish(axisMsg);

    // Publish button data
    var buttonMsg = new ROSLIB.Message({
        data: Array.from(gamepad_button)
    });
    buttonTopic.publish(buttonMsg);
}
