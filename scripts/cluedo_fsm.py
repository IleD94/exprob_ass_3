#!/usr/bin/env python

""" 
@package cluedo state machine
This node handles the states of the FSM
divided in: EXPLORATION, QUERY, ORACLE
"""
import rospy
import smach
import smach_ros
import random
import time
from erl2.srv import Oracle
# from erl2.msg import ErlOracle 
from cluedo.srv import Marker, HypothesisID
from armor_msgs.msg import *
from armor_msgs.srv import *
from armor_api.armor_client import ArmorClient
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

######Global declarations

suspects = ["MissScarlett", "ColonelMustard", "MrsWhite", "MrGreen", "MrsPeacock", "ProfPlum"]
weapons = ["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
rooms = ["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]
rooms_dictonary = {'Room1': [-4,-3], 'Room2': [-4,2], 'Room3': [-4,7], 'Room4':[5,-7] , 'Room5':[5,-3], 'Room6':[5,1]}
client = ArmorClient("cluedo", "ontology")
setgoal_client = None
aruco_sub = None
pub = None

ID_list = []
marker_detected = False
marker_id = []

def from_url_to_my_item (url):
    my_string = str (url)
    my_string = my_string.replace ('<http://www.emarolab.it/cluedo-ontology#', '')
    my_item = my_string.replace ('>', '')
    my_item = my_item.replace ('[', '')
    my_item = my_item.replace (']', '')
    return my_item

def winning_sequence (HP):
    who_url = client.query.objectprop_b2_ind ('who', HP)
    who = from_url_to_my_item (who_url)
    what_url = client.query.objectprop_b2_ind ('what', HP)
    what = from_url_to_my_item (what_url)
    where_url = client.query.objectprop_b2_ind ('where', HP)
    where = from_url_to_my_item (where_url)
    #user_interface ("ENTRO NEL MAKER!!!!")
    return who , what , where


def marker_callback (msg):
    global marker_id, marker_detected
    marker_id = msg.ID
    marker_detected = True


def set_goal (coordinates):
	
    global setgoal_client
    actiongoal = MoveBaseGoal ()
    actiongoal.goal.target_pose.header.frame_id = "map"
    actiongoal.goal.target_pose.pose.orientation.w = 1
    actiongoal.goal.target_pose.pose.position.x = coordinates[0]
    actiongoal.goal.target_pose.pose.position.y = coordinates[1]    
    setgoal_client.send_goal(actiongoal)
    setgoal_client.wait_for_result()
    return []

def clbk_odom(msg):
	global position_

    	# position
	position_ = msg.pose.pose.position

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

    
        pub = rospy.Publisher('cluedo_ui', String, queue_size=10) 
        time.sleep(1)
        try:
            rospy.loginfo(msg)
            pub.publish(msg)
        except rospy.ROSInterruptException:
            pass
        
        

class hint_gen:
      """
      Class hint_gen manages the client of the hint_generator service 
      custom service: Hint

      """
      def __init__(self):
          rospy.wait_for_service('hint_generator')
          self.srv_hint_client = rospy.ServiceProxy('hint_generator', Marker)
      
      def hint_client(self, markerId):
          """
          /brief client of the serice hint_generator
          custom service: Hint
              uint32 ID
              ---
              string myID
              string hint
          @param req: uint32 ID
          @return string: myID a code for the source 
          @return string: hint an hint of kind who, where or what
          """
          global res
          try:  
              self.res = self.srv_hint_client (markerId)
              user_interface ('Your hint is: '+ self.res.oracle_hint)
              return self.res
          except rospy.ServiceException as e:
              print("Service call failed: %s"%e)


class oracle_query:
      """
      Class oracle_query manages the client of the compare_hypothesis service 
      custom service: HypotesisID

      """
      def __init__(self):
          rospy.wait_for_service('compare_hypothesis')
          self.srv_oracle_client = rospy.ServiceProxy('compare_hypothesis', Oracle)
      
      def oracle_client(self):
          """
          /brief client of the service compare_hypotesis
          custom service: HypothesisID
              uint32 ID
              ---
              bool success
          @param req: uint32 ID
          @return bool: true if the the elements are the same, false if they are not
          """    
          global oracle_res
          try:  
              self.oracle_res = self.srv_oracle_client ()
              return self.oracle_res
          except rospy.ServiceException as e:
              print("Service call failed: %s"%e)

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
    Here the robot chooses randomly rooms and reaches them, then it gets
    an hint and restart again this process until it has an hypothesis with 3 items,
    when this happens the state machines goes into the state 2 QUERY 
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['hint_collection'],
                            )

    def execute(self,userdata):
        rospy.loginfo('Executing state Exploration')
        global counter
        counter = 0
        get_random_room = random.choice(list(rooms_dictonary.items())) #forse meglio una ricerca sistematica delle stanze
        room, coordinates = get_random_room
        user_interface ("I'm going to the: "+ room)
        if room in rooms_dictonary:
            del rooms_dictonary [room]
        set_goal (coordinates)
        user_interface ("REACHED!")
        setgoal_client.cancel_all_goals()
        return 'hint_collection'

# define state2 LOOK_AROUND
class LookAround(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['check_hypo', 'hint_collection', 'go_around'],
                             output_keys=['myID']
                            )
        
    
    def execute(self,userdata):
        
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0.3
        pub.publish(twist_msg)
        while not marker_detected:
            time.sleep(0.1)
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
        counter = counter+1
        print (counter)
        hint = hint_gen ()
        res = hint.hint_client(marker_id)
        userdata.myID=res.ID
        key= res.key
        value = res.value
        if key == '' or key == 'when' or value == '' or value == '-1':
            user_interface('Malformed hint, the robot will discard this') 
            return 'hint_collection'
        else:
            add_hypothesis (userdata.myID, value, key)
            if counter > 5:
                return 'go_around'
            else:
                return 'check_hypo'
        
# define state3 Query
class Query(smach.State):
    """
    Class Query is the second state of the cluedo fsm.
    Here the robot checks if the hypothesis is complete and not inconsistent, talking
    with the armor server using the api of the armor client,
    if it is in this way it goes to the third state ORACLE to make the accusation and
    check if it is the winning hypothesis, otherwise it returns to the state1
    """
    def __init__(self):
    
        smach.State.__init__(self, 
                             outcomes=['go_around', 'go_to_oracle'],
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
              user_interface ('The HP'+str(ID)+' is INCONSISTENT')
              time.sleep(1)
              return 'go_around'
        else:
           user_interface ('The HP'+str(ID)+' is INCOMPLETE')
           return 'go_around'
        
#define state3 Oracle
class Oracle (smach.State):
    """
    Class Oracle is the third state of the cluedo fsm.
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
        rospy.loginfo('Executing state Oracle')
        user_interface ("I'm going to the Oracle room...")
        set_goal (0,-1)
        user_interface ("REACHED!")
        setgoal_client.cancel_all_goals()
        [killer, weapon, room] = winning_sequence (HP)
        user_interface ('I accuse! It was '+ killer +' with '+ weapon +' in '+ room+ '!')
        oracle = oracle_query()
        res = oracle.oracle_client()
        print (res.ID)
        print (userdata.myID)
        if res.ID == userdata.myID :
           user_interface ('Your hypotesis with ID: '+ userdata.myID + ' is: corrects. You WIN, Detective Bot!')
           user_interface (killer + ' killed Dr. Black with '+ weapon+ ' in the '+ room)   
           client.utils.save_ref_with_inferences("/root/Desktop/inferred_cluedo.owl")     
           return 'end' #end of the game
        else:
           user_interface ('Your hypotesis with ID: '+ userdata.myID + ' is: wrong!')
           user_interface ('You LOOSE, go on with your research, Detective Bot!')
           return 'go_around'                     

def main():
    rospy.init_node('cluedo_fsm')
    #togliere i publisher dai globali
    global sub_odom, pub, setgoal_client, pub, aruco_sub
    sub_odom = rospy.Subscriber('odom', Odometry, clbk_odom)
	#initialization of the publisher to the topic cmd_vel to set the velocity
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	#initialization of the publisher to the topic move_base/goal, it is an action to set the goal
    setgoal_client = rospy.SimpleActionClient('move_base', MoveBaseAction)
    aruco_sub = rospy.Subscriber ('aruco/marker_id', Int32, marker_callback)

	
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = ''
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('EXPLORATION', Exploration(),
                               transitions={'hint_collection':'LOOK_AROUND'})
        
        smach.StateMachine.add('LOOK_AROUND', LookAround(), 
                               transitions={'check_hypo':'QUERY',
                                            'hint_collection':'LOOK_AROUND',
                                            'go_around' : 'EXPLORATION'},
                               remapping={'myID':'sm_counter'})
                               
        smach.StateMachine.add('QUERY', Query(), 
                               transitions={'go_to_oracle':'ORACLE', 
                                            'hint_collection':'LOOK_AROUND'})
                               
        smach.StateMachine.add('ORACLE', Oracle(), 
                               transitions={'go_around':'EXPLORATION', 
                                            'end' : 'outcome4'},
                               remapping={'myID':'sm_counter'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    time.sleep (5)
    # Execute SMACH plan
    sm.execute()
    
    while not rospy.is_shutdown():
       rospy.spin()
       sis.stop()

if __name__ == '__main__':
    main()