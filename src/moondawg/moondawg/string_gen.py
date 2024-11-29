from std_msgs.msg import String

class StringGen():
    
    @staticmethod
    @staticmethod
    def movement_string(lspeed, rspeed) -> String:
        """
        Returns a string to send to the arduino to control the movement of the wheels.

        Parameters:
        lspeed: The speed of the wheels for the left side of the bot (0-180, 90 stopped)
        direction: The speed of the wheels for the right side of the bot (0-180, 90 stopped)
        
        Returns:
        String: ROS String message format
        """
        string = String()
        string.data = f"m,{lspeed},{rspeed}"
        return string
        
    @staticmethod
    def camera_arm_string(angle) -> String:
        """
        Returns a string to send to the arduino to control the camera arm position.

        Parameters:
        pitch: Servo position value between 0 and 180
        
        Returns:
        String: ROS String message format
        """
        string = String()
        string.data = f"a,1,{angle}"
        return string
