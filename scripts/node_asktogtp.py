#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import requests
import json
import yaml
import rospkg

TOPIC_response_gtp = "/gtpresponse"
TOPIC_request_gtp = "/gtprequest"

class AskToGPTNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('asktogpt', anonymous=True)

        # Publisher to publish the GPT response
        self.gpt_response_pub = rospy.Publisher(TOPIC_response_gtp, String, queue_size=10)

        # Subscriber to receive the requested text
        rospy.Subscriber(TOPIC_request_gtp, String, self.handle_request)

        # Load the API key and group configuration from the YAML file
        config = self.load_config()
        self.api_key = config.get("apiKey")

        # Choose the group (e.g., museo, hotel, etc.)
        group = rospy.get_param('~group', 'museo')  # Default to "museo"
        group_config = config.get(group, {})

        # Extract group-specific parameters
        self.assid = group_config.get("assid")
        self.treadId = group_config.get("treadid")
        self.userId = group_config.get("userid")
        self.PHPsessionid = group_config.get("pHPsessionid")
        # rospy.loginfo("Api key: {}".format(self.api_key ))
        # rospy.loginfo("Ass id: {}".format(self.assid))
        # rospy.loginfo("tread id: {}".format(self.treadId))
        # rospy.loginfo("userid: {}".format(self.userId))
        # rospy.loginfo("PHP s id: {}".format(self.PHPsessionid))
        

        # API endpoint and headers
        self.url = "https://sviluppo.manpronet.com:8443/ai_project/v10/controller.php"
           

        self.headers = {
            'Content-Type': 'application/json',
            'Cookie': 'PHPSESSID={}'.format(self.PHPsessionid) 
        }
    

        rospy.loginfo("asktogpt v.1.01 node started in {} mode and listening...".format(group))

    def load_config(self):
        # Use rospkg to find the path to the marrtino_package
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('marrtino_package')  # Trova il percorso del package
        config_file = rospy.get_param('~config_file', package_path + '/config/config.yaml')  # Costruisci il percorso

        # Load the YAML configuration file
        with open(config_file, 'r') as file:
            return yaml.safe_load(file)
            
    def handle_request(self, msg):
        # Function to handle the received message
        request_text = msg.data
        rospy.loginfo("Request received: {}".format(request_text))

        # Build the payload for the API request
        payload = json.dumps({
            "apiKey": self.api_key,
            "action": "addMessage",
            "message": request_text,
            "assistID": self.assid,
            "threadID": self.treadId,
            "db": "mpnet_its_pesaro",
            "user": self.userId,
            "idUser": 1
        })
      
       # print(payload)
        # Send the request to the GPT API
        try:
            response = requests.post(self.url, headers=self.headers, data=payload)
            response_text = response.text
            rospy.loginfo("GPT Response: {}".format(response_text))

            # Publish the GPT response
            self.gpt_response_pub.publish(response_text)
        except Exception as e:
            rospy.logerr("Error in GPT request: {}".format(e))

if __name__ == '__main__':
    try:
        node = AskToGPTNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
