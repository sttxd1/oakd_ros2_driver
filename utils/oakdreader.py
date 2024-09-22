# import depthai
# for device in depthai.Device.getAllAvailableDevices():
#     print(f"{device.getMxId()} {device.state}")
    
import depthai as dai

# List all available devices
for device_info in dai.Device.getAllAvailableDevices():
    print(f"Device MXID: {device_info.getMxId()}")
    
