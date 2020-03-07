# %%


from pprzlink.lib.v2.python.pprzlink import ivy
import time

class Status:
    def __init__(self):
        self.north = 0 # y in gazebo
        self.east = 0  # x in gazebo

    def update_position(self, north, east):
        self.north = north
        self.east = east


def waypoint_moved(spl: str, status: Status):
    status.north = spl[2]
    status.east = spl[3]


opts = {
    "NED_POS": waypoint_moved
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

i.bind_raw(process_incoming_message, '(44 NED_POS.*)')

#while True:
    #print(str(status.x) + " " + str(status.y) + " " + str(status.z))
