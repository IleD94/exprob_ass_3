#!/usr/bin/env python

""" 
@package exp_assignemtn3, node cluedo_fsm
This node handles the states of the FSM
divided in: EXPLORATION, LOOK AROUND, QUERY, ORACLE
"""
import rospy
import smach
import smach_ros
import random
import time
import actionlib
from std_srvs.srv import Empty, EmptyResponse
from diagnostic_msgs.msg import KeyValue
#from exp_assignment3.msg import ErlOracle 
from exp_assignment3.srv import Marker, MarkerResponse, Oracle, OracleResponse, MoveArm, MoveArmResponse
from armor_msgs.msg import *
from armor_msgs.srv import *
from armor_api.armor_client import ArmorClient
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import math

######Global declarations

suspects = ["MissScarlett", "ColonelMustard", "MrsWhite", "MrGreen", "MrsPeacock", "ProfPlum"]
weapons = ["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
rooms = ["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]
rooms_dictionary = {'Room1': [-4,-3], 'Room2': [-4,2], 'Room3': [-4,7], 'Room4':[5,-7] , 'Room5':[5,-3], 'Room6':[5,1]}

client = ArmorClient("cluedo", "ontology")
setgoal_client = None
aruco_sub = None
pub = None
marker_list = []
ID_list = []
marker_detected = False
marker_id = []
coordinates = []
notreached = False
attempts=0
position_ = Point()
desired_position_ = Point()
desired_position_.x = None
desired_position_.y = None
desired_position_.z = 0
global_counter = []
srv_oracle_client = None
srv_hint_client = None

def clbk_odom(msg):
    """
    /brief callback of the subscriber sub_odom, to the topic 'odom'. 
    Here it sets the global variable position_ with data from the odometry.
    Every rate the current position of the robot is known.
    @param: msg
    #return: None
    """
    global position_
	# position
    position_ = msg.pose.pose.position

def error_distance_calculation (x,y):
    """
    /brief this function calculates the error between the current position of the robot
    and the goal. It returns true if the error is under a specific threshold, otherwise
    it returns false.
    @param: x, coordinate of the goal
    @param: y, coordinate of the goal
    @return: Bool
    """
    global position_, desired_position_
    desired_position_.x = x
    desired_position_.y = y
    err_pos = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
    print ("We are almost there! Our distance from the goal is:")			
    print (err_pos)
    if(err_pos < 0.4): #threshold under which we have reached the goal
        print ('You reach your goal!')
        return True
    else: 
        return False

def from_url_to_my_item (url):
    """
    /brief this function trasforms a url into a string with only the individual of
    the class returned from the query of the armor_service.
    @param: url 
    @return: my_item
    """
    my_string = str (url)
    my_string = my_string.replace ('<http://www.emarolab.it/cluedo-ontology#', '')
    my_item = my_string.replace ('>', '')
    my_item = my_item.replace ('[', '')
    my_item = my_item.replace (']', '')
    return my_item

def winning_sequence (HP):
    """
    /brief this function takes who, where and what from the answer of the armor_service 
    for a specific hypothesis and transfrom the url to a string, taking just the individual
    of the class
    @param: HP
    @return: who, what, where, list of strings.
    """
    who_url = client.query.objectprop_b2_ind ('who', HP)
    who = from_url_to_my_item (who_url)
    what_url = client.query.objectprop_b2_ind ('what', HP)
    what = from_url_to_my_item (what_url)
    where_url = client.query.objectprop_b2_ind ('where', HP)
    where = from_url_to_my_item (where_url)
    return who , what , where


def marker_callback (msg):
    """
    /brief this function is the callback of the subscriber to the topic /marker_publisher/detected_id.
    when an aruco marker is detected the id is received in this callback. If the id of the marker is
    major of 40it is discarded. Otherwise it is put in a list with the othe found ids, in a global counter.
    @param: msg
    @return: None
    """
    global marker_id, marker_detected, global_counter
    marker_id = msg.data
    if marker_id<=40 and marker_id>=11:
        if marker_id not in global_counter:
            marker_detected = True
            global_counter.append (marker_id)
            print (global_counter)
        else:
            print (global_counter)
    

def set_goal (x,y):
    """
    /brief this function sets the goal coordinates for the movebase action.
    @param: x, coordinate x of the goal
    @param: y, coordinate y of the goal
    @return void list.
    """
    global setgoal_client
    actiongoal = MoveBaseGoal ()
    actiongoal.target_pose.header.frame_id = "map"
    actiongoal.target_pose.pose.orientation.w = 1
    actiongoal.target_pose.pose.position.x = x
    actiongoal.target_pose.pose.position.y = y   
    setgoal_client.send_goal(actiongoal)
    return []

def make_ind_of_class_disjoint (class_name):
        """
        /brief Disjoint all individuals of a class in an ontology by comunication with the armor server.
    
        Args:
            ind_name (str): individual to be disjointed to the class.
            class_name (str): individual will be disjointed to this class. It will be created a new class if it does not exist.
    
        Returns:
            bool: True if ontology is consistent, else False
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error
    
        Note:
            It returns the boolean consistency state of the ontology. This value is not updated to the last operation
            if you are working in buffered reasoner or manipulation mode!
        """
        try:
            res = client.call('DISJOINT', 'IND', 'CLASS', [class_name])
    
        except rospy.ServiceException as e:
            raise ArmorServiceCallError(
                "Service call failed upon adding individual {0} to class {1}: {2}".format(ind_name, class_name, e))
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        if res.success:
            return res.is_consistent
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)
            
def user_interface(msg):
        """
        /brief userinterface publisher. It sends a screen that is shown on the screen
        @param msg: String
        @return : None
        """
        pub = rospy.Publisher('cluedo_ui', String, queue_size=10) 
        time.sleep(1)
        try:
            rospy.loginfo(msg)
            pub.publish(msg)
        except rospy.ROSInterruptException:
            pass
        
        

def add_hypothesis (ID, item, key):
    """
    /brief this function add items in the class HYPOTHESIS in the cluedo ontology
    and add an individual to the object property who, where and what, depending on 
    the kind of hint that it has received from the hint_generator. At the end it
    saves the changes and synchronize the reasoner using armor api.
    @param ID: uint32 with the number of the source
    @param item: the hint to add in the cluedo ontology as where, who or what instance 
    """
    global HP #forse va tolto
    
    HP = "HP"+str(ID)
    #myID = rospy.get_param (HP) #CONSISTENT
    client.manipulation.add_ind_to_class(HP, "HYPOTHESIS")
    client.manipulation.add_dataprop_to_ind("hasID", "HP"+str(ID), "STRING", str(ID))
    client.manipulation.add_objectprop_to_ind(key, HP, item)
    client.utils.apply_buffered_changes()
    client.utils.sync_buffered_reasoner()
    return (HP)
 

# define state1 EXPLORATION
class Exploration(smach.State):
    """
    Classe Exploration it is the first state of the cluedo fsm
    Here the robot chooses randomly rooms and reaches them, if 
    in its way finds some hints it goes to the query state, otherwise
    it reaches the goal and pass to the look_around state
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['hint_collection', 'go_around', 'end'],
                            )

    def execute(self,userdata):
        rospy.loginfo('Executing state Exploration')
        global counter, notreached, global_counter, coordinates, rooms_dictionary
        if len(global_counter)==30:
            user_interface ("You collected all marker hints, but you couldn't find a solution")
            return 'end'
        #it stopped before reaching the goal to catch a hint from a marker. It restore the goal
        #after get the hint
        if notreached: 
            set_goal (coordinates[0],coordinates[1])
            error_flag = error_distance_calculation (coordinates[0],coordinates[1])
            print (error_flag)
            while marker_detected or (not (error_flag)) :
                error_flag = error_distance_calculation (coordinates[0],coordinates[1])
                if error_flag:
                    user_interface ("REACHED!")
                    notreached = False
                    break
                elif marker_detected == True: 
                    notreached =True
                    break
            counter = counter + 1
        else:
            counter = 0
            #Some marker could not be reachable in the zero pose, so we try to catch 22 hints and then go in the investigation position
            #to start again to go around to each room to catch the missing ones.
            if len (global_counter)<22:
                res = moveit_client('zero')
            else:
                rooms_dictionary = {'Room1': [-4,-3], 'Room2': [-4,2], 'Room3': [-4,7], 'Room4':[5,-7] , 'Room5':[5,-3], 'Room6':[5,1]}
                res = moveit_client('investigation')
            get_random_room = random.choice(list(rooms_dictionary.items()))
            room, coordinates = get_random_room
            user_interface ("I'm going to the: "+ room)
            if room in rooms_dictionary:
                del rooms_dictionary [room]
            set_goal (coordinates[0],coordinates[1])
            error_flag = error_distance_calculation (coordinates[0],coordinates[1])
            print (error_flag)
            while marker_detected or (not (error_flag)) :
                error_flag = error_distance_calculation (coordinates[0],coordinates[1])
                if error_flag:
                    user_interface ("REACHED!")
                    notreached = False
                    break
                elif marker_detected == True:
                    notreached = True
                    break
        setgoal_client.cancel_all_goals()   
        return 'hint_collection'
        

# define state2 LOOK_AROUND
class LookAround(smach.State):
    """
    Class LookAround, it is the second state of the cluedo fsm
    Here the robot goes around in the room and catches them, then goes to
    the query state to check the consistency and the completeness. 
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['check_hypo', 'hint_collection', 'go_around'],
                             output_keys=['myID']
                            )
        
    
    def execute(self,userdata):
        global marker_detected, counter, global_counter, notreached, attempts, srv_hint_client
        if not notreached:
            res = moveit_client('investigation')
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.linear.y = 0
            twist_msg.angular.z = 0.5
            pub.publish(twist_msg)
            counter = counter+1
            time.sleep(0.5)
        if marker_detected and marker_id != None:
            print ('ho detectacto')
            twist_msg = Twist()
            twist_msg.angular.z = 0
            marker_detected = False
            pub.publish(twist_msg)
            res =  srv_hint_client (global_counter[-1])
            user_interface ('Your hint is: ')
            user_interface (str(res.oracle_hint))
            userdata.myID=res.oracle_hint.ID
            key= res.oracle_hint.key
            value = res.oracle_hint.value
            if key == '' or key == 'when' or value == '' or value == '-1':
                user_interface('Malformed hint, the robot will discard this') 
                return 'hint_collection'
            else:
                add_hypothesis (res.oracle_hint.ID, value, key)   
                return 'check_hypo'
        else:
            if notreached:
                return 'go_around'
            else:
                attempts=attempts + 1 
                if attempts>6:
                    attempts = 0
                    return 'go_around'
                else:
                    return 'hint_collection'
        
        
# define state3 Query
class Query(smach.State):
    """
    Class Query is the second state of the cluedo fsm.
    Here the robot checks if the hypothesis is complete and not inconsistent, talking
    with the armor server using the api of the armor client,
    if it is in this way it goes to the third state ORACLE to make the accusation and
    check if it is the winning hypothesis, otherwise it returns to the state1. If the
    hint was found on the way, it returns to the state 1, in order to reach the room
    object of the goal.
    """
    def __init__(self):
    
        smach.State.__init__(self, 
                             outcomes=['hint_collection', 'go_to_oracle'],
                             input_keys=['myID']
                            )
        
    def execute(self, userdata):
        rospy.loginfo('Executing state QUERY')  
        make_ind_of_class_disjoint ("PERSON")
        make_ind_of_class_disjoint ("WEAPON")
        make_ind_of_class_disjoint ("PLACE")     
        inconsistent_list = client.query.ind_b2_class("INCONSISTENT")
        complete_list = client.query.ind_b2_class("COMPLETED")
        inconsistent_str = str (inconsistent_list)
        print (inconsistent_str)
        complete_str = str (complete_list)
        print (complete_str)
        ID = userdata.myID
        if (complete_str.find ("HP"+str(ID)) != -1):
           #user_interface ('The HP'+str(ID)+' is COMPLETE')
           if (inconsistent_str.find ("HP"+str(ID)) == -1):
              if ID not in ID_list:
                user_interface ('The HP'+str(ID)+' is CONSISTENT')
                ID_list.append(ID)
                time.sleep(1)
                return 'go_to_oracle'
              else:
                  user_interface ("The HP"+str(ID)+ " is CONSISTENT, but we've already checked and it is not the winning one")
                  return 'hint_collection'
           else:
              user_interface ('The HP'+str(ID)+' is INCONSISTENT')
              time.sleep(1)
              return 'hint_collection'
        else:
           user_interface ('The HP'+str(ID)+' is INCOMPLETE')
           return 'hint_collection'
        
#define state3 Oracle
class myOracle (smach.State):
    """
    Class Oracle is the fourth state of the cluedo fsm.
    It checks if the hypothesis received has the winning ID code, asking
    to the oracle that compares it with the ID of the winner.
    If it is not the winner. It return to the state 1 otherwise it ends the
    state machine.
    """
    
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['go_around', 'end'],
                             input_keys=['myID']
                            )

    def execute(self, userdata):
        global srv_oracle_client
        rospy.loginfo('Executing state Oracle')
        user_interface ("I'm going to the Oracle room...")
        res = moveit_client('zero')
        set_goal (0,-1)
        error_flag = error_distance_calculation (0,-1)
        while not (error_flag) :
                error_flag = error_distance_calculation (0,-1)
        user_interface ("REACHED!")
        setgoal_client.cancel_all_goals()
        [killer, weapon, room] = winning_sequence (HP)
        user_interface ('I accuse! It was '+ killer +' with '+ weapon +' in '+ room+ '!')
        res = srv_oracle_client()
        print (res.ID)
        print (userdata.myID)
        if res.ID == userdata.myID :
           user_interface ('Your hypotesis with ID: '+ str(userdata.myID) + ' is: correct. You WIN, Detective Bot!')
           user_interface (killer + ' killed Dr. Black with '+ weapon+ ' in the '+ room)   
           client.utils.save_ref_with_inferences("/root/Desktop/inferred_cluedo.owl")     
           return 'end' #end of the game
        else:
           user_interface ('Your hypotesis with ID: '+ str(userdata.myID) + ' is: wrong!')
           user_interface ('You LOOSE, go on with your research, Detective Bot!')
           return 'go_around'                     

def main():
    rospy.init_node('cluedo_fsm')
    #togliere i publisher dai globali
    global pub, setgoal_client, pub, aruco_sub, moveit_client,sub_odom, srv_oracle_client,srv_hint_client
	#initialization of the publisher to the topic cmd_vel to set the velocity
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # odom subscriber
    sub_odom = rospy.Subscriber('odom', Odometry, clbk_odom)
	#initialization of the publisher to the topic move_base/goal, it is an action to set the goal
    setgoal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    setgoal_client.wait_for_server()
    srv_oracle_client = rospy.ServiceProxy('/oracle_solution', Oracle)
    rospy.wait_for_service('/oracle_solution')
    srv_hint_client = rospy.ServiceProxy('/oracle_hint', Marker)
    rospy.wait_for_service('/oracle_hint')
    aruco_sub = rospy.Subscriber ('/marker_publisher/detected_id', Int32, marker_callback)
    moveit_client =rospy.ServiceProxy('myarm_pose', MoveArm)
    rospy.wait_for_service('myarm_pose')

	
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = ''
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('EXPLORATION', Exploration(),
                               transitions={'hint_collection':'LOOK_AROUND', 
                                            'go_around' : 'EXPLORATION',
                                            'end':'outcome4'},)
        
        smach.StateMachine.add('LOOK_AROUND', LookAround(), 
                               transitions={'check_hypo':'QUERY',
                                            'hint_collection':'LOOK_AROUND',
                                            'go_around' : 'EXPLORATION'},
                               remapping={'myID':'sm_counter'})
                               
        smach.StateMachine.add('QUERY', Query(), 
                               transitions={'go_to_oracle':'ORACLE', 
                                            'hint_collection':'LOOK_AROUND'},
                               remapping={'myID':'sm_counter'})
        smach.StateMachine.add('ORACLE', myOracle(), 
                               transitions={'go_around':'EXPLORATION', 
                                            'end' : 'outcome4'},
                               remapping={'myID':'sm_counter'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    time.sleep (1)
    # Execute SMACH plan
    sm.execute()
    
    while not rospy.is_shutdown():
       rospy.spin()
       sis.stop()

if __name__ == '__main__':
    main()
