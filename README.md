# NonBlockingModbusMaster
This library is a modification of the Doc Walker's ModbusMaster library.  To access Modbus devices an RS232 to RS485 converter is necessary.

This library's features include:-  
* Completely non-blocking, with suitably sized serial TX and RX buffers  
* Works with any Stream object.   
* Can access multiple device IDs.   
* No callback methods are necessary.   
* Has a retry() method for uses when the command times out (or other error)   
* Has oneTimeDelay() method to insert an extra pre-command delay when switching between ID's   
* Has a simple command chaining facility to group commands together in a single executable block.    
* Supports optional results processing functions called when command completes.   
* Runs on UNO with software serial while maintaining responsive Serial processing.   

# How-To
See [NonBlockingModbusMaster Tutorial](https://www.forward.com.au/pfod/ArduinoProgramming/NonBlockingModbusMaster/index.html)    

# Software License
(c)2025 Forward Computing and Control Pty. Ltd.  
NSW Australia, www.forward.com.au  
This code is not warranted to be fit for any purpose. You may only use it at your own risk.  
This code may be freely used for both private and commercial use  
Provide this copyright is maintained.  
Also see LICENSE.txt for original ModbusMaster license.  

# Revisions
Version 1.0.2 added UNO_modbus_simple_chaining example.  
Version 1.0.1 corrected printHex arg and retry() return.  
Version 1.0.0 initial release. readHoldingRegisters is the only Modbus command fully tested due to lack of other hardware. 
