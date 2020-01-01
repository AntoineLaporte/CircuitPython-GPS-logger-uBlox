import board
import storage
import analogio
 
analogin = analogio.AnalogIn(board.A2) 
 
voltage = (analogin.value * 3.3) / 65536
if voltage > 2:
	storage.remount("/", True )
else:
	storage.remount("/", False )