import can
import struct

bus = can.interface.Bus("can0", interface="socketcan")

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

node_id = 0 # must match `<odrv>.axis0.config.can.node_id`. The default is 0.
cmd_id = 0x01 # heartbeat command ID
message_id = (node_id << 5 | cmd_id)


for msg in bus:
  if msg.arbitration_id == message_id:
      error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
      break
print(error, state, result, traj_done)

