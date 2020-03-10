import numpy as np
import os
import cv2
import time

os.sys.path.append('/home/parallels/paparazzi-tudelft/python-cnns/pprzlink/lib/v2/python/')

from pprzlink import ivy
from pprzlink.message import PprzMessage

import torch
from torch.autograd import Variable

import image_preprocessing

class Status:
    def __init__(self):
        self.distance = 0
        self.collisions = 0
        self.repositioning = False
        self.opts = {"TRAINING_STATE": self.update_state}

    def process_incoming_message(self, source, pprz_message):
        #print(pprz_message)
        spl = pprz_message.split(" ")
        if len(spl) > 1 and spl[1] in self.opts:
            self.opts[spl[1]](spl)

    def update_state(self, spl):
        self.distance = float(spl[2])
        self.collisions = int(spl[3])
        self.repositioning = int(spl[4]) == 1



class PaparazziGym:

    def __init__(self):
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "protocol_whitelist;file,rtp,udp"
        self.preprocessor = image_preprocessing.PreprocessImage()

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
        speedidx = int(np.mod(action, 3))
        headingidx = int(np.floor(action / 3))
        return self.speeds[speedidx], self.headings[headingidx]
        
    def getSnapshot(self):
        _, img = self.cap.read()

        img = self.preprocessor.process(img)
        
        return torch.tensor(np.array(img, dtype = np.float32)).unsqueeze(0).unsqueeze(0)

    def reset(self):
        message = ivy.PprzMessage("datalink", "TRAINING_ACTION")

        message["reset"] = 1
        message["speed"] = 0
        message["heading"] = 0
        message["ac_id"] = 44

        self.ivy.send_raw_datalink(message)

        time.sleep(.1)
        self.collisionSeries = 0

        return self.getSnapshot()

    def step(self, action):
        while (self.status.repositioning):
            None

        speed, heading = self.unwrapAction(action)
        message = ivy.PprzMessage("datalink", "TRAINING_ACTION")

        message["reset"] = 0
        message["speed"] = float(speed)
        message["heading"] = float(heading)
        message["ac_id"] = 44
        
        self.ivy.send_raw_datalink(message)
        
        collisions = self.status.collisions
        distance = self.status.distance

        time.sleep(.11)

        # read picture
        next_state = self.getSnapshot()
        
        # read collision
        newColl = (self.status.collisions - collisions)
        # give reward

        newDist = (self.status.distance - distance)

        self.collisionSeries += newColl
        print(self.status.repositioning)
        is_done = self.collisionSeries >= 3 or self.status.repositioning

        if is_done:
            self.collisionSeries = 0

        reward = newColl * -1000 + newDist
        print(reward)
        return next_state, reward, is_done
