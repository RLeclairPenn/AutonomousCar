from detectionClass import detectionClass
from Arduino_Communicator import ArduinoCommunicator
import time

lowH = 60
highH = 101
lowS = 236
highS = 255
lowV = 168
highV = 255

detection = detectionClass(lowH, lowS, lowV, highH, highS, highV)
communicator = ArduinoCommunicator(0x8)

while True:
    x, y = detection.doAnalysis()
    if x != -1 and y != -1:
        cxp, cyp = detection.return_cxp_cyp(y, x)
        communicator.write_float_to_register(cxp, 0) # Write cxp to reg 0
        communicator.write_float_to_register(cyp, 1) # Write cyp to reg 1
        communicator.trigger_method(100)
        time.sleep(0.1)
    # Uncomment this x, y = detection.doAnalysisTest()
    # 
    # UT if x != -1 and y != -1:
        # UT cxp, cyp = detection.return_cxp_cyp(y, x)
    # UT else:
        # UT print("Cone Not There")

