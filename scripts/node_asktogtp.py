#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import requests
import json
import yaml
import rospkg

TOPIC_response_gtp = "/gtpresponse"
TOPIC_request_gtp = "/gtprequest"
TOPIC_sendphoto_gtp = "/gtpphoto"
TOPIC_set_group = "/gtpsetgroup"
TOPIC_speech = "/social/speech/to_speak"
TOPIC_speechstatus = "/social/speech/status"
TOPIC_log_msg = "/log_msg"

class AskToGPTNode:
    
    def __init__(self):
        # Initialize the node
        rospy.init_node('asktogpt', anonymous=True)

        # Publisher for GPT response
        self.gpt_response_pub = rospy.Publisher(TOPIC_response_gtp, String, queue_size=10)
        self.speech_pub = rospy.Publisher(TOPIC_speech, String, queue_size=1, latch=True)
        self.log_msg_pub = rospy.Publisher(TOPIC_log_msg, String, queue_size=1, latch=True)

        # Subscriber for requested text
        rospy.Subscriber(TOPIC_request_gtp, String, self.handle_request)
        # Subscriber for group set requests
        rospy.Subscriber(TOPIC_set_group, String, self.update_group)
        rospy.Subscriber(TOPIC_sendphoto_gtp, String, self.send_photo)
        # Load the configuration file and initialize with the default group
        self.config = self.load_config()
        self.group = rospy.get_param('~group', 'hotel')  # Default to "museo"
        self.initialize_group(self.group)
        self.log_msg_pub.publish("asktoai v.1.02 node started in {} mode and listening...".format(self.group))
        rospy.loginfo("asktoai v.1.02 node started in {} mode and listening...".format(self.group))

    def setlanguage(self, msg):
        # Placeholder function to set language
        #print '/social/speech/language %s' % (msg)
        self.language_pub.publish(msg)

    def speech(self, msg):
        # print '/social/speech/to_speak %s' % (msg)
        self.speech_pub.publish(msg)

    def load_config(self):
        # Use rospkg to find the path to the marrtino_package
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('marrtino_package')  # Find package path
        config_file = rospy.get_param('~config_file', package_path + '/config/config.yaml')  # Build config path
        
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
        self.iduser = group_config.get("iduser")
        self.message = group_config.get("message")
        self.dscgtp = group_config.get("dscgpt")
                
       
        # API endpoint and headers
        self.headers = {
            'Content-Type': 'application/json',
            'Cookie': 'PHPSESSID={}'.format(self.PHPsessionid)
        }

        # Activate a new session and get the thread ID
        self.log_msg_pub.publish("Group initialized to: {}".format(group))
        
        self.treadId = self.activatesession()
        rospy.loginfo("Group initialized to: {}".format(group))
        self.speech(self.dscgtp)

    def activatesession(self):
        # Activate a new session
        payload = json.dumps({
            "apiKey": self.api_key,
            "action": "addConversation",
            "message": self.message,
            "db": "mpnet_its_pesaro",
            "user": self.userId,
            "idUser": self.iduser
        })
       # rospy.loginfo("Payload being sent to GPT API: {}".format(payload))
        try:
            json_response = requests.post(self.url, headers=self.headers, data=payload)
            json_response.raise_for_status()  # Check for HTTP errors
            data = json.loads(json_response.text)

            # Retrieve the 'id' field within 'data'
            treadid = data['data']['id']
            rospy.sleep(3)

            # Send a message to the newly created conversation
            payload = json.dumps({
                "apiKey": self.api_key,
                "action": "addMessage",
                "message": self.message,
                "assistID": self.assid,
                "threadID": treadid,
                "db": "mpnet_its_pesaro",
                "user": self.userId,
                "idUser": self.iduser
            })

            response = requests.post(self.url, headers=self.headers, data=payload)
            response.raise_for_status()
            rospy.sleep(3)
            self.log_msg_pub.publish("Ok Start session TreadId {}".format(treadid))
            
            rospy.loginfo("ok start session")
            return treadid

        except requests.exceptions.RequestException as e:
            rospy.logerr("HTTP Request failed: {}".format(e))
            return None

    def handle_request(self, msg):
        # Handle the received message
        request_text = msg.data
        rospy.loginfo("Request received: {}".format(request_text))
        if request_text == "PONG!":
            rospy.loginfo("Pong")
        else:
            rospy.loginfo(request_text)
            # Build the payload for the API request
            payload = json.dumps({
                "apiKey": self.api_key,
                "action": "addMessage",
                "message": request_text,
                "assistID": self.assid,
                "threadID": self.treadId,
                "db": "mpnet_its_pesaro",
                "user": self.userId,
                "idUser": self.iduser
            })
            headers = {
                'Content-Type': 'application/json',
                'Cookie': 'PHPSESSID={}'.format(self.PHPsessionid)
                }

      
            # Send the request to the GPT API
            try:
                response = requests.post(self.url, headers=headers, data=payload)
                response.raise_for_status()  # Check for HTTP errors
                response_text = response.text
                rospy.loginfo("AI Response: {}".format(response_text))

                # Publish the GPT response
                self.gpt_response_pub.publish(response_text)
            except Exception as e:
                rospy.logerr("Error in AI request: {}".format(e))

    def update_group(self, msg):
        # Handle the group update request
        new_group = msg.data
        self.log_msg_pub.publish("Updating group to: {}".format(new_group))
        rospy.loginfo("Updating group to: {}".format(new_group))
        
        self.speech(format(new_group))
        self.initialize_group(new_group)


    def send_photo(self, msg):
        # Handle the group update request
        request_text =  msg.data
        self.log_msg_pub.publish("Send photo ")
        rospy.loginfo("Send photo ")
      
        payload = {'action': 'addMessage',
            'apiKey': self.api_key,
            'db': 'mpnet_its_pesaro',
            'user': 'AssMuseo',
            'idUser': self.iduser ,
            'threadID': self.treadId,
            'message': request_text,
            'assistID': self.assid}
        files=[
            ('files[]',('biglietto.png',open('/home/robot/src/marrtino_package/image/biglietto.png','rb'),'image/png'))
        ]
        headers = {
             'Cookie': 'PHPSESSID={}'.format(self.PHPsessionid)
        }
        # Open the file in binary mode and prepare it for upload
        

       
        # Make the POST request
        response = requests.post(self.url, headers=headers, data=payload, files=files)
        response_text = response.text
                
        # Check for success and print the response
        if response.status_code == 200:
                
            self.log_msg_pub.publish("Ok send photo succesful")
        else:
            self.log_msg_pub.publish("Send photo failed with status code {}".format(response.status_code))
                                                            
        
        rospy.loginfo("AI Response: {}".format(response_text))
        # Publish the GPT response
        self.gpt_response_pub.publish(response_text)
     
 
if __name__ == '__main__':
    try:
        node = AskToGPTNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
