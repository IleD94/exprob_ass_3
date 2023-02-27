#!/usr/bin/env python

""" 
@package cluedo default settings of the
cluedo ontology.
This node sets all the classes and individuals
in the OWL file by the comunication with the Armor Client
"""

import rospy
import random
import time
#from cluedo.srv import Hint
from armor_msgs.msg import *
from armor_msgs.srv import *
from armor_api.armor_client import ArmorClient

suspects = ["missScarlett", "colonelMustard", "mrsWhite", "mrGreen", "mrsPeacock", "profPlum"]
weapons = ["candlestick", "dagger", "leadPipe", "revolver", "rope", "spanner"]
rooms = ["conservatory", "lounge", "kitchen", "library", "hall", "study", "bathroom", "diningRoom", "billiardRoom"]


class DefaultSettings:
     """
     Class DefaultSettings manages the setting of the 
     cluedo ontology, it loads the ontology, adds classes and individuals to those classes. 
     It has all the functions necessary to comunicate with armor server, 
     using armor client api in order to add individuals to classes,  
     save changes and sincronyze the reasoner 
     """

     def __init__(self): 
         print("Ready to start the game")

     def load_ontology (self):
         """
         /brief function to load the ontology in the armor server and to set the reasoner
         """
         global client
         client = ArmorClient("cluedo", "ontology")
         client.utils.load_ref_from_file("/root/Desktop/cluedo_ontology.owl", "http://www.emarolab.it/cluedo-ontology",
                                True, "PELLET", True, False)  # initializing with buffered manipulation and reasoning
         client.utils.mount_on_ref()
         client.utils.set_log_to_terminal(True)
     
     def add_item_into_class (self):
         """
         /brief function to add the individuals to each specific class
         """
         for x in suspects:
            client.manipulation.add_ind_to_class(x, "PERSON")
            print ("Added", x,"to person")
         for x in weapons:
            client.manipulation.add_ind_to_class(x, "WEAPON")
            print ("Added", x, "WEAPON")
         for x in rooms:
            client.manipulation.add_ind_to_class(x, "PLACE")
            print ("Added", x, "to Place")

     
     def changes_and_apply (self):
         """
         /brief function to save changes and reason on those
         """
         client.utils.apply_buffered_changes()
         time.sleep (3)
         client.utils.sync_buffered_reasoner()
         #for x in suspects:
          #  if client.query.check_ind_exists(x):
            #   print ("Item", x,"is present")
         #for x in weapons:
          #  if client.query.check_ind_exists(x):
             #  print ("Item", x,"is present")
         #for x in rooms:
            #if client.query.check_ind_exists(x):
          #     print ("Item", x,"is present")
        
     
                    
                    
if __name__ == "__main__":
    rospy.init_node('ontology_settings')
    #time.sleep (10)
    settings = DefaultSettings ()
    settings.load_ontology()
    settings.add_item_into_class()
    settings.changes_and_apply()        
     
    while not rospy.is_shutdown():  
       rospy.spin()
