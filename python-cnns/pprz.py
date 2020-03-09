# %%


from pprzlink.lib.v2.python.pprzlink import ivy
import time
import numpy as np

class Status:
    def __init__(self):
        self.distance = 0 # y in gazebo
        self.collisions = 0  # x in gazebo
        self.state = 0

    def update_position(self, north, east):
        self.north = north
        self.east = east


def waypoint_moved(spl: str, status: Status):
    status.north = spl[2]
    status.east = spl[3]


def training_state(spl: str, status: Status):
    status.distance = float(spl[2])
    status.collisions = int(spl[3])
    status.state = int(spl[4])

opts = {
    "TRAINING_STATE": training_state
}


def process_incoming_message(source, pprz_message: str):
    print(pprz_message)
    spl = pprz_message.split(" ")
    if len(spl) > 1 and spl[1] in opts:
        opts[spl[1]](spl, status)


status = Status()

i = ivy.IvyMessagesInterface(
    agent_name=None,
    start_ivy=True,
    verbose=True
)

i.bind_raw(process_incoming_message, '(44 TRAINING_STATE.*)')





# while True:
#     time.sleep(1)

#     message = ivy.PprzMessage("datalink", "TRAINING_ACTION")

#     message["reset"] = 0
#     message["speed"] = float(2.)
#     message["heading"] = float(60.)
#     message["ac_id"] = 44

#     out = i.send_raw_datalink(message)
#     print(out)

# %%
