#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String

class GPTResponseListener:
    def __init__(self):
        # Sottoscrizione al topic 'gptresponse'
        self.gpt_response_sub = rospy.Subscriber('gtpresponse', String, self.callback)

    def callback(self, msg):
        try:
            # Decodifica il messaggio JSON
            json_data = json.loads(msg.data)
            
            # Stampa l'intero JSON decodificato in modo leggibile
            rospy.loginfo("Messaggio decodificato ricevuto:")
            rospy.loginfo(json.dumps(json_data, indent=4, ensure_ascii=False))

            # Estrazione e stampa di tutti i campi
            status = json_data.get("status", "N/A")
            msg_field = json_data.get("msg", "N/A")
            error = json_data.get("error", "N/A")
            data = json_data.get("data", {})
            action = json_data.get("action", "N/A")

            # Campi specifici all'interno di 'data'
            macro_vr = data.get("macro_vr", [])
            emotion = data.get("emotion", [])
            language = data.get("language", [])
            speech = data.get("speech", [])
            head = data.get("head", [])
            gesture = data.get("gesture", [])
            wait = data.get("wait", [])
            url = data.get("url", [])
            message = data.get("message", "N/A")

            # Log di tutti i campi
            rospy.loginfo(f"Status: {status}")
            rospy.loginfo(f"Msg: {msg_field}")
            rospy.loginfo(f"Error: {error}")
            rospy.loginfo(f"Action: {action}")
            rospy.loginfo(f"Macro VR: {macro_vr}")
            rospy.loginfo(f"Emotion: {emotion}")
            rospy.loginfo(f"Language: {language}")
            rospy.loginfo(f"Speech: {speech}")
            rospy.loginfo(f"Head: {head}")
            rospy.loginfo(f"Gesture: {gesture}")
            rospy.loginfo(f"Wait: {wait}")
            rospy.loginfo(f"URL: {url}")
            rospy.loginfo(f"Message: {message}")
            
        except json.JSONDecodeError:
            rospy.logerr("Errore nella decodifica del messaggio JSON")

if __name__ == '__main__':
    rospy.init_node('gpt_response_listener', anonymous=True)
    listener = GPTResponseListener()
    
    # Mantieni il nodo in esecuzione
    rospy.spin()
