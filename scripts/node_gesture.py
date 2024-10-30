#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy 
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from threading import Timer
import math
import time 


# SOCIAL
# 
TOPIC_emotion = "social/emotion"
TOPIC_speech = "social/speech/to_speak"
TOPIC_pan = "pan_controller/command"
TOPIC_tilt = "tilt_controller/command"
TOPIC_spalla_dx_rot = "/spalladx_controller/command"
TOPIC_spalla_dx_fle = "/spalladxj_controller/command"
TOPIC_gomito_dx = "/gomitodx_controller/command"
TOPIC_hand_right = "/handdx_controller/command"
TOPIC_spalla_sx_rot = "/spallasx_controller/command"
TOPIC_spalla_sx_fle = "/spallasxj_controller/command"
TOPIC_gomito_sx = "/gomitosx_controller/command"
TOPIC_hand_left = "/handsx_controller/command"
#eof social

TOPIC_gesture = "/social/gesture"

# publisher
emotion_pub = rospy.Publisher(TOPIC_emotion, String, queue_size=1,   latch=True)
pan_pub = rospy.Publisher(TOPIC_pan, Float64, queue_size=1,   latch=True)
tilt_pub = rospy.Publisher(TOPIC_tilt, Float64, queue_size=1,   latch=True)
speech_pub =rospy.Publisher(TOPIC_speech, String, queue_size=1,   latch=True)
# 
spalla_dx_rot_pub = rospy.Publisher(TOPIC_spalla_dx_rot, Float64, queue_size=1,   latch=True)
spalla_dx_fle_pub = rospy.Publisher(TOPIC_spalla_dx_fle, Float64, queue_size=1,   latch=True)
gomito_dx_pub = rospy.Publisher(TOPIC_gomito_dx, Float64, queue_size=1,   latch=True) 
spalla_sx_rot_pub = rospy.Publisher(TOPIC_spalla_sx_rot, Float64, queue_size=1,   latch=True)
spalla_sx_fle_pub = rospy.Publisher(TOPIC_spalla_sx_fle, Float64, queue_size=1,   latch=True)
gomito_sx_pub = rospy.Publisher(TOPIC_gomito_sx, Float64, queue_size=1,   latch=True)
hand_right_pub = rospy.Publisher(TOPIC_hand_right, Float64, queue_size=1,   latch=True)
hand_left_pub = rospy.Publisher(TOPIC_hand_left, Float64, queue_size=1,   latch=True)



def gesture_indica_sinistra():
    left_shoulder_flexion(20)
    left_shoulder_rotation(20)
    left_elbow(40)
    left_hand(-70)

def gesture_zero():
    emotion("startblinking")
    head_position("front")
    emotion("normal")
    left_shoulder_rotation(15)
    right_shoulder_rotation(15)
    left_shoulder_flexion(-70)
    right_shoulder_flexion(-70)
    left_hand(0)
    right_hand(0)
    left_elbow(10)
    right_elbow(10)

def gesture_hello():
    emotion("startblinking")
    head_position("front")
    spalla_rotazione_dx(2.6166666666666667)
    spalla_flessione_dx(1.57)
    gomito_dx(2.8783333333333334)
    hand_right(2.6166666666666667)
    say('ciao')
    emotion("speak")
    spalla_rotazione_sx(0.17444444444444446)
    time.sleep(1)
    emotion("happy")
    for count in range(2):
        spalla_flessione_sx(3.14)
        hand_left(3.663333333333333)
        gomito_sx(2.0933333333333333)
        time.sleep(2)
        spalla_flessione_sx(3.837777777777778)
        hand_left(3.488888888888889)
        gomito_sx(2.7911111111111113)
        time.sleep(2)
    emotion("normal")
    gesture_zero()
    
def gesture_down():
    head_position("front")
    emotion("blinking")
    emotion("normal")
   

def gesture_up():
    right_elbow(30)
 

def gesture_pos1():
    right_elbow(30)
    left_elbow(30)
    right_hand(60)
    left_hand(60)

def gesture_pos2():
    right_elbow(20)
    left_elbow(20)
    right_hand(30)
    left_hand(30)

def gesture_poszero():
    right_shoulder_flexion(-70)
    left_shoulder_flexion(-70)
    right_shoulder_rotation(30)
    left_shoulder_rotation(20)
    right_elbow(30)
    left_elbow(30)
    right_hand(80)
    left_hand(80)

def gesture_init():
    right_shoulder_flexion(-70)
    left_shoulder_flexion(-70)
    right_shoulder_rotation(0)
    left_shoulder_rotation(0)
    right_elbow(0)
    left_elbow(0)
    right_hand(0)
    left_hand(0)

def gesture_anim():
    head_position("front")
    emotion("blinking")
    emotion("normal")
    for i in range(1,2):
        gesture_poszero()
        time.sleep(1) # Sleep for 3 secondstime.sleep(1)
        gesture_pos1()
        time.sleep(1) # Sleep for 3 seconds
        gesture_pos2()
        time.sleep(1) # Sleep for 3 seconds
        gesture_zero()
        time.sleep(1) # Sleep for 3 seconds

def reset_gesture():
    gesture_anim()


def hand_identify():
    right_shoulder_flexion(-60)
    right_shoulder_rotation(90)
    right_elbow(70)
    right_hand(-70)

def point_left():
    left_shoulder_flexion(20)
    left_shoulder_rotation(20)
    left_elbow(30)
    left_hand(-70)

    

def point_right():
    right_shoulder_flexion(20)
    right_shoulder_rotation(20)
    right_elbow(20)
    right_hand(-70)


def greeting():
    right_shoulder_flexion(20)
    right_shoulder_rotation(30)
    right_hand(-120)
    for count in range(5):
        right_elbow(80)
        time.sleep(2)
        right_elbow(30)
        time.sleep(2)



def point_up():
    head_position("up")
    right_shoulder_flexion(45)
    right_shoulder_rotation(40)
    right_elbow(60)
    right_hand(-80)

def check_ticket():
    right_shoulder_flexion(-65)
    right_shoulder_rotation(30)
    right_hand(-90)
    for count in range(2):
        right_elbow(60)
        time.sleep(0.5)
        right_elbow(50)
        time.sleep(0.5)


def hello():
    right_shoulder_flexion(-30)
    right_shoulder_rotation(10)
    right_hand(-90)
    for count in range(2):
        right_elbow(60)
        time.sleep(0.5)
        right_elbow(50)
        time.sleep(0.5)
    left_shoulder_flexion(-30)
    left_shoulder_rotation(10)
    left_hand(-80)
    for count2 in range(2):
        left_elbow(60)
        time.sleep(0.5)
        left_elbow(50)
        time.sleep(0.5)


### SOCIAL ####
################



def say(msg):
    print('speech %s' %(msg))
    speech_pub.publish(msg)

def head_status(msg):
    print('social/emotion %s' %(msg))

def emotion(msg):
    #
    print('social/emotion %s' %(msg))
    emotion_pub.publish(msg)


def spalla_rotazione_dx(msg):
    # valori da -0.5  0 0.5 
    print('spalla_rotazione_dx: %s' %(msg))
    spalla_dx_rot_pub.publish(msg)

def spalla_flessione_dx(msg):
    print('spalla_flessione_dx: %s' %(msg))
    spalla_dx_fle_pub.publish(msg)

def gomito_dx(msg):
    #
    print('gomito_dx: %s' %(msg))
    gomito_dx_pub.publish(msg)


def spalla_rotazione_sx(msg):
    # valori da -0.5  0 0.5 
    print('spalla_rotazione_sx: %s'  %(msg))
    spalla_sx_rot_pub.publish(msg)

def spalla_flessione_sx(msg):
    #
    # 1.918 = +40  -> Limite up
    print('spalla_flessione_sx: %s'  %(msg))
    spalla_sx_fle_pub.publish(msg)

def gomito_sx(msg):
    #
    print('gomito_sx: %s' %(msg))
    gomito_sx_pub.publish(msg)
    
def hand_left(msg):
    #
    print('hand_left: %s' %(msg))
    hand_left_pub.publish(msg)    
 
def hand_right(msg):
    #
    print('hand_right: %s' %(msg))
    hand_right_pub.publish(msg)    

def pan(msg):
    # valori da -0.5  0 0.5 
    print('Pan Position: %s' %(msg))
    pan_pub.publish(msg)

def tilt(msg):
    #
    print('Tilt Position: %s' %(msg))
    tilt_pub.publish(msg)

def head_position(msg):
    print('Head : %s' %(msg))
    if (msg == 'front'):
        pan_pub.publish(0)
        tilt_pub.publish(0)
    if (msg == 'left'):
        pan_pub.publish(0.5)
        tilt_pub.publish(0)
    if (msg == 'half_left'):
        pan_pub.publish(0.25)
        tilt_pub.publish(0)
    if (msg == 'right'):
        pan_pub.publish(-0.5)
        tilt_pub.publish(0)
    if (msg == 'half_right'):
        pan_pub.publish(-0.25)
        tilt_pub.publish(0)
    if (msg == 'down'):
        pan_pub.publish(0)
        tilt_pub.publish(0.5)
    if (msg == 'up'):
        pan_pub.publish(0)
        tilt_pub.publish(-0.5)
# create function en english and value degree
#############################################
# Calcoliamo i gradi relativi 150 = Centro

def right_shoulder_rotation(vdeg):
    vrad = DEG2RAD(150 + vdeg)
    spalla_rotazione_dx(vrad)

def left_shoulder_rotation(vdeg):
    vdeg = -vdeg
    vrad = DEG2RAD(150 + vdeg)
    spalla_rotazione_sx(vrad)

def right_shoulder_flexion(vdeg):
    vdeg = -vdeg
    vrad = DEG2RAD(150 + vdeg)
    spalla_flessione_dx(vrad)

def left_shoulder_flexion(vdeg):
   
    vrad = DEG2RAD(150 + vdeg)
    spalla_flessione_sx(vrad)

def right_elbow(vdeg):
    vrad = DEG2RAD(150 + vdeg)
    gomito_dx(vrad)
    
def left_elbow(vdeg):
    vdeg = -vdeg
    vrad = DEG2RAD( 150 + vdeg)
    gomito_sx(vrad)
  
def right_hand(vdeg):
    vdeg = -vdeg
    vrad = DEG2RAD(150 + vdeg)
    hand_right(vrad)
    
def left_hand(vdeg):
    vrad = DEG2RAD( 150 + vdeg)
    hand_left(vrad)

# Angle functions

def DEG2RAD(a):
    return a*math.pi/180.0

def RAD2DEG(a):
    return a/math.pi*180.0
#def get_nro_of_face:
#    return 1

###########################################
# time in seconds
TIME_DELAY = 5
rospy.set_param("face_reset_timer",TIME_DELAY)

def reset_gesture():    
    gesture_anim()
    rospy.loginfo("Gesture: reset anim")
    t = Timer(TIME_DELAY,reset_gesture)



def restart_timer():
    global t
    t = Timer(TIME_DELAY,reset_gesture)
        
def start_timer():
    restart_timer()
    rospy.loginfo("Gesture: start timer")
    t.start()

def stop_timer():
    t.cancel()

def callback_gesture(data):
    gesture = data.data
    if (gesture == 'init'):
        gesture_init()
    if (gesture == 'gesture'):
        gesture_anim()
    if (gesture == 'zero'):
        gesture_zero()
    if (gesture == 'down'):
        gesture_down()
    if (gesture == 'up'):
        gesture_up()
    if (gesture == 'start'):
        start_timer()
    if (gesture == 'hello'):
        hello()
    if (gesture == 'indica_sinistra'):
        gesture_indica_sinistra()
    if (gesture == 'point_left'):
        point_left()
    if (gesture == 'point_right'):
        point_right()
    if (gesture == 'point_up'):
        point_up()
    if (gesture == 'hand_identify'):
        hand_identify()
    if (gesture == 'hand_indentify'):
        hand_identify()
    if (gesture == 'greeting'):
        greeting()
    if (gesture == 'check_ticket'):
        check_ticket()
        
    if (gesture == 'stop'):
        start_timer()

def listener():
    rospy.init_node("gesture_node")
    rospy.loginfo("Gesture node start")
    rospy.Subscriber(TOPIC_gesture,String,callback_gesture)
    rospy.spin()

listener()
