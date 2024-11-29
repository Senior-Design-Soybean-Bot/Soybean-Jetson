// Parameters - change these to match the Pi
var ip = "192.168.4.255"; // IP of the Pi
var port = "9090"; // Port of webserver node

var gamepad_axis_prev = "null";
var gamepad_button_prev = "null";
var url_string = `ws://${ip}:${port}`;

// Connect to and set up ros bridge
var ros = new ROSLIB.Ros({
    url: url_string
});

ros.on('connection', function() {
    console.log('Connected to ROSBridge!');
    
    var connectionStatus = new ROSLIB.Message({
        data: 0
    });

    setInterval(() => connectionTopic.publish(connectionStatus), 250);
});

ros.on('error', function(error) {
    console.log('Error connecting to ROSBridge:', error);
});

ros.on('close', function() {
    setTimeout(() => ros.connect(url_string), 5000); // Retry connection every 5000 ms
});

// Define topics
var connectionTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/connection_status',
    messageType: 'std_msgs/Byte'
});
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
var gpsTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/fix',
    messageType: 'sensor_msgs/NavSatFix'
});
var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});
const NUM_IMAGES = 4;
let imageContainer = document.getElementById('image-container');
let imageElements = [];

for (let i = 0; i < NUM_IMAGES; i++) {
    let imgDiv = document.createElement('div');
    imgDiv.className = 'image-box';
    let img = document.createElement('img');
    img.style.width = '300px';
    imgDiv.appendChild(img);
    imageContainer.appendChild(imgDiv);
    imageElements.push(img);
}
var imageTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/image/compressed',
    messageType : 'sensor_msgs/CompressedImage'
});


axisTopic.subscribe(function(message) {
    document.getElementById('axis-display').innerHTML = message.data;
});
buttonTopic.subscribe(function(message) {
    document.getElementById('button-display').innerHTML = message.data;
});
gpsTopic.subscribe(function(message) {
    var lat = message.latitude.toFixed(6);
    var lon = message.longitude.toFixed(6);
    var gpsString = `Lat: ${lat}, Lon: ${lon}`;
    document.getElementById('gps-display').innerHTML = gpsString;
});
imageTopic.subscribe(function(message){
    const imageDat = "data:image/jpeg;base64," + message.data;
    
    for (let i = imageElements.length - 1; i > 0; i--) {
        imageElements[i].src = imageElements[i-1].src;
    }
    imageElements[0].src = imageData;
}); 

// Connect gamepad
window.addEventListener("gamepadconnected", function(e) {
    console.log("Gamepad connected!");
    setInterval(readControllerData, 75); // Read from controller every 75 ms
});

let lastAxisData = null;
let lastButtonData = null;

// Function to be called every time the controlled is read from
function readControllerData() {
    var gamepad = navigator.getGamepads()[0]; // Assuming the first connected gamepad
    
    if (!gamepade || !ros.isConnected) {
        return;
    }

    gamepad_axis = gamepad.axes.map(axis => parseInt(axis.toFixed(2)*100))
    gamepad_button = gamepad.buttons.map(button => button.value = parseInt((button.value.toFixed(2)*100)));

    var axisData = new ROSLIB.Message({
        data: gamepad_axis
    });

    axisTopic.publish(axisData);
    
    var buttonData = new ROSLIB.Message({
        data: gamepad_button
    });

    buttonTopic.publish(buttonData);
}
