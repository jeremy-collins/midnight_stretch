import os
from twilio.rest import Client
import rospy
import numpy as np

rospy.init_node("voice_call")

account_sid = None # REPLACE WITH TWILIO ACCOUNT INFO
auth_token = None # REPLACE WITH TWILIO ACCOUNT INFO

client = Client(account_sid, auth_token)

call = client.calls.create(
    to = "+14043850124", #number calling to CHANGE THIS 
    from_ =  None, # REPLACE WITH TWILIO ACCOUNT PHONE #
    url = "http://demo.twilio.com/docs.voice.xml"
)
print(call.sid)
