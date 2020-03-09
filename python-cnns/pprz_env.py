import numpy as np
import os
import cv2
import time
from pprzlink.lib.v2.python.pprzlink import ivy
#from pprzlink.lib.v2.python.pprzlink.messages import PprzMessage


import image_preprocessing

class Status:
    def __init__(self):
        self.distance = 0 # y in gazebo
        self.collisions = 0  # x in gazebo
        self.state = 0
        self.opts = {"TRAINING_STATE": self.update_state}

    def process_incoming_message(self, source, pprz_message: str):
        print(pprz_message)
        spl = pprz_message.split(" ")
        if len(spl) > 1 and spl[1] in self.opts:
            self.opts[spl[1]](spl)

    def update_state(self, spl):
        self.distance = float(spl[2])
        self.collisions = int(spl[3])
        self.state = int(spl[4])



class PaparazziGym:

    def __init__(self):
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "protocol_whitelist;file,rtp,udp"

        self.speeds = [.5, 1., 1.5]
        self.headings = [-60., 0., 60.]
        self.cap = cv2.VideoCapture('/home/parallels/paparazzi-tudelft/sw/tools/rtp_viewer/rtp_5000.sdp')

        self.status = Status()

        self.collisionSeries = 0

        self.ivy = ivy.IvyMessagesInterface(
            agent_name=None,
            start_ivy=True,
            verbose=True
        )

        self.ivy.bind_raw(self.status.process_incoming_message, '(44 TRAINING_STATE.*)')

    def quit(self):
        self.cap.release()

    def unwrapAction(self, action):
        speedidx = np.mod(action, 3)
        headingidx = np.floor(action / 3)
        return self.speeds[speedidx], self.headings[headingidx]
        
    def getSnapshot(self):
        _, img = self.cap.read()

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        img = img.mean(-1, keepdims = True)
        img = np.transpose(img, (2, 0, 1))
        img = img.astype('float32') / 255.

        return img

    def reset(self):
        message = ivy.PprzMessage("datalink", "TRAINING_ACTION")

        message["reset"] = 1
        message["speed"] = 0
        message["heading"] = 0
        message["ac_id"] = 44

        self.ivy.send_raw_datalink(message)

        time.sleep(.1)
        self.collisionSeries = 0

    def step(self, action):
        speed, heading = self.unwrapAction(action)
        message = ivy.PprzMessage("datalink", "TRAINING_ACTION")

        message["reset"] = 0
        message["speed"] = float(speed)
        message["heading"] = float(heading)
        message["ac_id"] = 44
        
        self.ivy.send_raw_datalink(message)
        
        collisions = self.status.collisions
        distance = self.status.distance

        time.sleep(.1)

        # read picture
        next_state = self.getSnapshot()
        # read collision
        newColl = (self.status.collisions - collisions)
        # give reward

        newDist = (self.status.distance - distance)

        self.collisionSeries += newColl
        is_done = self.collisionSeries >= 3

        if is_done:
            self.collisionSeries = 0

        return next_state, newColl * -100 + newDist * .1, is_done
