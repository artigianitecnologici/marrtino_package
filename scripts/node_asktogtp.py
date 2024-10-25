#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import requests
import json
import yaml
import rospkg

TOPIC_response_gtp = "/gtpresponse"
TOPIC_request_gtp = "/gtprequest"
TOPIC_set_group = "/gtpsetgroup"
TOPIC_speech = "/social/speech/to_speak"
TOPIC_speechstatus = "/social/speech/status"

class AskToGPTNode:
    
    def __init__(self):
        # Initialize the node
        rospy.init_node('asktogpt', anonymous=True)

        # Publisher to publish the GPT response
        self.gpt_response_pub = rospy.Publisher(TOPIC_response_gtp, String, queue_size=10)
        self.speech_pub =  rospy.Publisher(TOPIC_speech, String, queue_size=1,   latch=True)
        self.language_pub = rospy.Publisher(TOPIC_language, String, queue_size=1,   latch=True)

        # Subscriber to receive the requested text
        rospy.Subscriber(TOPIC_request_gtp, String, self.handle_request)
        # Subscriber to receive the group set requests
        rospy.Subscriber(TOPIC_set_group, String, self.update_group)

        # Load the configuration file and initialize with the default group
        self.config = self.load_config()
        self.group = rospy.get_param('~group', 'museo')  # Default to "museo"
        self.initialize_group(self.group)

        rospy.loginfo("asktogpt v.1.01 node started in {} mode and listening...".format(self.group))

    def setlanguage(msg):
        print '/social/speech/language %s' % (msg)
        self.language_pub.publish(msg)

    def speech(msg):
        print '/social/speech/to_speak %s' % (msg)
        self.speech_pub.publish(msg)


    def load_config(self):
        # Use rospkg to find the path to the marrtino_package
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('marrtino_package')  # Trova il percorso del package
        config_file = rospy.get_param('~config_file', package_path + '/config/config.yaml')  # Costruisci il percorso

        # Load the YAML configuration file
        with open(config_file, 'r') as file:
            return yaml.safe_load(file)

    def initialize_group(self, group):
        # Load the group-specific configuration
        group_config = self.config.get(group, {})
        
        # Extract group-specific parameters
        self.api_key = self.config.get("apiKey")
        self.assid = group_config.get("assid")
        self.userId = group_config.get("userid")
        self.PHPsessionid = group_config.get("pHPsessionid")
        self.url = group_config.get("urlgtp")
        # API endpoint and headers
       
        self.headers = {
            'Content-Type': 'application/json',
            'Cookie': 'PHPSESSID={}'.format(self.PHPsessionid)
        }

        # Activate a new session and get the thread ID
        self.treadId = self.activatesession()

        rospy.loginfo("Group initialized to: {}".format(group))

    def activatesession(self):
        payload = json.dumps({
            "apiKey": self.api_key,
            "action": "addConversation",
            "db": "mpnet_its_pesaro",
            "user": self.userId,
            "idUser": 6
        })
        
        try:
            json_response = requests.post(self.url, headers=self.headers, data=payload)
            json_response.raise_for_status()  # Verifica se ci sono errori HTTP
            data = json.loads(json_response.text)

            # Recupera il campo 'id' all'interno di 'data'
            treadid = data['data']['id']
            rospy.sleep(3)

            # Invia un messaggio alla conversazione appena creata
            payload = json.dumps({
                "apiKey": self.api_key,
                "action": "addMessage",
                "message": "Presentati ed effettua il Login con queste coppie di chiave valore, Login:AssMuseo Password:12345678",
                "assistID": self.assid,
                "threadID": treadid,
                "db": "mpnet_its_pesaro",
                "user": self.userId,
                "idUser": 6
            })

            response = requests.post(self.url, headers=self.headers, data=payload)
            response.raise_for_status()
            rospy.sleep(3)
            print("ok start")
            return treadid

        except requests.exceptions.RequestException as e:
            rospy.logerr(f"HTTP Request failed: {e}")
            return None

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
      
        # Send the request to the GPT API
        try:
            response = requests.post(self.url, headers=self.headers, data=payload)
            response.raise_for_status()  # Check for HTTP errors
            response_text = response.text
            rospy.loginfo("GPT Response: {}".format(response_text))

            # Publish the GPT response
            self.gpt_response_pub.publish(response_text)
        except Exception as e:
            rospy.logerr("Error in GPT request: {}".format(e))

    def update_group(self, msg):
        # Handle the group update request
        new_group = msg.data
        rospy.loginfo("Updating group to: {}".format(new_group))
        self.speech("gruppo,")
        self.speech(format(new_group))
        self.initialize_group(new_group)

if __name__ == '__main__':
    try:
        node = AskToGPTNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
