#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import requests
import json
import yaml
import os

class AskToGPTNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('asktogpt', anonymous=True)

        # Publisher to publish the GPT response
        self.gpt_response_pub = rospy.Publisher('gptresponse', String, queue_size=10)

        # Subscriber to receive the requested text
        rospy.Subscriber('gtprequest', String, self.handle_request)

        # Load the API key from the YAML file
        config = self.load_config()
        self.api_key = config.get("apiKey")

        # API endpoint and headers
        self.url = "https://sviluppo.manpronet.com:8443/ai_project/v09/controller.php"
        self.headers = {
            'Content-Type': 'application/json',
            'Cookie': 'PHPSESSID=50la5tns4ule95udg6in7ko5aa'
        }

        rospy.loginfo("asktogpt node started and listening...")

    def load_config(self):
        # Load the YAML configuration file from the correct path
        config_file = rospy.get_param('~config_file') #  , '~/robot/src/marrtino_package/script/config.yaml')
        config_file = os.path.expanduser(config_file)  # Expand the ~ to the full home directory path
        with open(config_file, 'r') as file:
            return yaml.safe_load(file)

    def handle_request(self, msg):
        # Function to handle the received message (added the missing colon)
        request_text = msg.data
        rospy.loginfo("Request received: {}".format(request_text))

        # Build the payload for the API request
        payload = json.dumps({
            "apiKey": self.api_key,
            "action": "addMessage",
            "message": request_text,
            "assistID": "asst_vg5orqR32POATtTNfVMO3AZV",
            "threadID": "thread_XST1ratS0serYLDx4LphpYsK",
            "db": "mpnet_its_pesaro",
            "user": "monand",
            "idUser": 1
        })

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
