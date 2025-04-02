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

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass


bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
    data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
    is_extended_id=False
))

for msg in bus:
    if msg.arbitration_id == (node_id << 5 | 0x01): # 0x01: Heartbeat
        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        if state == 8: # 8: AxisState.CLOSED_LOOP_CONTROL
            break

bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
    data=struct.pack('<ff', 1.0, 0.0), # 1.0: velocity, 0.0: torque feedforward
    is_extended_id=False
))

for msg in bus:
    if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
        pos, vel = struct.unpack('<ff', bytes(msg.data))
        print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")


bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
    data=struct.pack('<ff', 0.0, 0.0), # 0.0: velocity, 0.0: torque feedforward
    is_extended_id=False
))

for msg in bus:
    if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
        pos, vel = struct.unpack('<ff', bytes(msg.data))
        print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")


bus.shutdown()
