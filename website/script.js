// Parameters - change these to match the Pi
const robotIP = '127.0.0.1';  // Using localhost since that's what we confirmed
const port = '9090';
const websocketURL = `ws://${robotIP}:${port}`;

// Initialize variables
let gamepad_axis_prev = null;
let gamepad_button_prev = null;

// Single ROS connection
const ros = new ROSLIB.Ros({
    url: websocketURL
});

// Connection event handlers
ros.on('connection', function() {
    console.log('Connected to ROSBridge!');
    document.getElementById('rosbridge-status').textContent = 'Connected';

    const connectionStatus = new ROSLIB.Message({
        data: 0
    });

    setInterval(() => connectionTopic.publish(connectionStatus), 250);
});

ros.on('error', function(error) {
    console.log('Error connecting to ROSBridge:', error);
    document.getElementById('rosbridge-status').textContent = 'Error';
});

ros.on('close', function() {
    document.getElementById('rosbridge-status').textContent = 'Disconnected';
    setTimeout(() => ros.connect(websocketURL), 5000);
});

// Define topics
const connectionTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/connection_status',
    messageType: 'std_msgs/Byte'
});

const axisTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/gamepad_axis',
    messageType: 'std_msgs/Int8MultiArray'
});

const buttonTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/gamepad_button',
    messageType: 'std_msgs/Int8MultiArray'
});

const gpsTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/fix',
    messageType: 'sensor_msgs/NavSatFix'
});

// Image handling setup
const NUM_IMAGES = 4;
const imageContainer = document.getElementById('image-container');
const imageElements = [];

for (let i = 0; i < NUM_IMAGES; i++) {
    const imgDiv = document.createElement('div');
    imgDiv.className = 'image-box';
    const img = document.createElement('img');
    img.style.width = '300px';
    imgDiv.appendChild(img);
    imageContainer.appendChild(imgDiv);
    imageElements.push(img);
}

const imageTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/camera/image/compressed',
    messageType: 'sensor_msgs/CompressedImage'
});

// Topic subscriptions
axisTopic.subscribe(function(message) {
    document.getElementById('left-x').textContent = message.data[0];
    document.getElementById('left-y').textContent = message.data[1];
    document.getElementById('right-x').textContent = message.data[2];
    document.getElementById('right-y').textContent = message.data[3];
});

buttonTopic.subscribe(function(message) {
    document.getElementById('controller-message').textContent = 'Button pressed: ' + message.data;
});

gpsTopic.subscribe(function(message) {
    document.getElementById('latitude').textContent = message.latitude.toFixed(6);
    document.getElementById('longitude').textContent = message.longitude.toFixed(6);
    document.getElementById('altitude').textContent = message.altitude.toFixed(2);
});

imageTopic.subscribe(function(message) {
    const imageData = "data:image/jpeg;base64," + message.data;

    for (let i = imageElements.length - 1; i > 0; i--) {
        imageElements[i].src = imageElements[i-1].src;
    }
    imageElements[0].src = imageData;
});

// Gamepad handling
window.addEventListener("gamepadconnected", function(e) {
    console.log("Gamepad connected!");
    document.getElementById('controller-status').textContent = 'Connected';
    setInterval(readControllerData, 75);
});

window.addEventListener("gamepaddisconnected", function(e) {
    console.log("Gamepad disconnected!");
    document.getElementById('controller-status').textContent = 'Disconnected';
});

function readControllerData() {
    const gamepad = navigator.getGamepads()[0];

    if (!gamepad || !ros.isConnected) {
        return;
    }

    const gamepad_axis = gamepad.axes.map(axis => parseInt(axis.toFixed(2) * 100));
    const gamepad_button = gamepad.buttons.map(button => parseInt((button.value.toFixed(2) * 100)));

    axisTopic.publish(new ROSLIB.Message({
        data: gamepad_axis
    }));

    buttonTopic.publish(new ROSLIB.Message({
        data: gamepad_button
    }));
}
