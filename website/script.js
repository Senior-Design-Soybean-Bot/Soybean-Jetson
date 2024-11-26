// Parameters - change these to match the Pi
var ip = "192.168.48.128"; // IP of the Pi
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
var diagnosticTopic = new ROSLIB.Topic({ // Diagnostics not yet used
    ros: ros,
    name: '/diagnostics',
    messageType: 'diagnostic_msgs/DiagnosticArray'
});
var imageTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/image',
    messageType: 'sensor_msg/Image'
});
var gpsTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/fix',
    messageType: 'sensor_msgs/NavSatFix'
});
var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});
var lastImageTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/last_captured_image',
    messageType : '2x2_array/LastCapturedImages'
});

var imageIndex = 1;

// Subscribe to topics
/* 
imageTopic.subscribe(function(message) {
    document.getElementById("video_out").src = "data:image/jpeg;base64," + message.data;
}); 
*/

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
lastImageTopic.subscribe(function(message){
    for (var i = 0; i < message.images.length; i++){
        var imageId = 'last_captured_image_' + imageIndex;
        var filenmaeId = 'filename_' + imageIndex;

        document.getElementById(imageId).src = "data:image/jepg;base64," + message.images[i];
        document.getElementById(filenmaeId).textContent = message.filenames[i];

        imageIndex = (imageIndex % 4) + 1;
    }

}); 

// Connect gamepad
window.addEventListener("gamepadconnected", function(e) {
    console.log("Gamepad connected!");
    setInterval(readControllerData, 75); // Read from controller every 75 ms
});

// Function to be called every time the controlled is read from
function readControllerData() {
    var gamepad = navigator.getGamepads()[0]; // Assuming the first connected gamepad
    
    if (gamepad == undefined) {
	    return;
	}

    if (!ros.isConnected) {
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
