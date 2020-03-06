# %%


from pprzlink.lib.v2.python.pprzlink import ivy


class Status:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

    def update_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def waypoint_moved(spl: str, status: Status):
    status.x = spl[5]
    status.y = spl[6]
    status.z = spl[7]


opts = {
    "WAYPOINT_MOVED": waypoint_moved
}


def process_incoming_message(source, pprz_message: str):
    print(pprz_message)
    spl = pprz_message.split(" ")
    opts[spl[1]](spl, status)


status = Status()

i = ivy.IvyMessagesInterface(
    agent_name=None,
    start_ivy=True,
    verbose=True,
    ivy_bus="192.168.2.47:2010"
)

i.bind_raw(process_incoming_message, '(.*)')
