'''
@file      main.py
@brief     client side application code to transfer selected firmware binary file 
           as packets.
@author    AvinasheeTech
'''

import asyncio
from bleak import BleakClient,BleakScanner
import datetime
import os

#Advertised OTA Service UUID
ADVERTISED_UUID  = ["6f9742f3-97b2-b594-f343-74a44f52d0d2"]     #OTA Service 
MANUF_ID = 0x02E5                     #Company Identifier from Bluetooth SIG Document
MANUF_BYTES = 0x782184926910          #Base Mac Address of Device
#characteristic uuids for ota service
OTA_DATA_UUID    = "0000ee02-0000-1000-8000-00805f9b34fb"  #OTA Data characteristic
OTA_CONTROL_UUID = "0000ee01-0000-1000-8000-00805f9b34fb"  #OTA Control characteristic

#flags for ota operation
OTA_NOP =         bytearray.fromhex("00")
OTA_REQUEST =     bytearray.fromhex("01")
OTA_REQUEST_ACK = bytearray.fromhex("02")
OTA_REQUEST_NAK = bytearray.fromhex("03")
OTA_DONE =        bytearray.fromhex("04")
OTA_DONE_ACK =    bytearray.fromhex("05")
OTA_DONE_NAK =    bytearray.fromhex("06")

#queue for data transfer
queue = asyncio.Queue()

'''
@brief  search for our device amongst available nearby devices
@retval instance of the device found
'''
async def __search_for_device__():

    server_device = None

    discovered_devices_and_advertisement_data = await BleakScanner.discover(return_adv=True)
    for device, adv_dat in discovered_devices_and_advertisement_data.values():
        print("scanned device:"+str(device.address))
        for key, value in adv_dat.manufacturer_data.items():  #manuf data = company id + base mac address
            if((key==MANUF_ID) & (("0x"+value.hex())==hex(MANUF_BYTES)) & (adv_dat.service_uuids == ADVERTISED_UUID)): #filter with base mac addr and service uuid 
                print("device found...................with address "+str(device.address) +" and uuid " + str(adv_dat.service_uuids))
                server_device = device

    
    assert server_device is not None, "device not found................................." 

    return server_device


'''
@brief  callback function to be called on characteristic data notified
@note   sends a queue data as ack or nak to main function 
        based on the notification received by the server device 
'''
async def __ota_callback__(sender:int,data:bytearray):

    if(data == OTA_REQUEST_ACK):
        print("ota start successful. starting to send data.....................")
        await queue.put("ack")
    elif(data == OTA_REQUEST_NAK):
        print("ota start failed!!!!!!!!!!!!!!!!!!!!!")
        await queue.put("nak")
    elif(data == OTA_DONE_ACK):
        print("ota completion successful.stopping now........................")
        await queue.put("ack")
    elif(data == OTA_DONE_NAK):
        print("ota completion failed!!!!!!!!!!!!!!!!!!!!")
        await queue.put("nak")
    else:
        print("sender:{} data:{}".format(sender,data))
        
'''
@brief  main function that takes care of sending characteristic value and firmware 
        chunks of data once OTA begins.
@param  path of binary file
'''
async def __ota_main__(file_path):

    begin_time = datetime.datetime.now()
    firmware_data = []

    try:
        device = await __search_for_device__()
        if(os.path.exists(file_path)):
            async with BleakClient(device) as client:

                #start notification for ota control characteristic
                await client.start_notify(OTA_CONTROL_UUID,__ota_callback__)

                #set data packet size
                packet_size = 253        #mtu packet = 256 bytes = 3 bytes header + 253 bytes actual payload 
                print("packet size for ble data transfer-"+str(packet_size))

                #write packet size
                await client.write_gatt_char(OTA_DATA_UUID,packet_size.to_bytes(2,"little"),True)

                #send ota write request
                await client.write_gatt_char(OTA_CONTROL_UUID,OTA_REQUEST)

                #wait for response 
                await asyncio.sleep(1)

                #check if successful ack received
                if await queue.get() == "ack":
                    #get chunks of binary file ready to send
                    with open(file_path,'rb') as file:
                        while packet_data:= file.read(packet_size):
                            firmware_data.append(packet_data)
                    

                    #send chunks of data
                    for index,ota_data in enumerate(firmware_data):
                        print(f"sending data packet:{index+1}/{len(firmware_data)}")
                        await client.write_gatt_char(OTA_DATA_UUID,ota_data,True)

                else:
                    print("ESP32 did not acknowledge the OTA request.")

                
                #send ota stop request
                await client.write_gatt_char(OTA_CONTROL_UUID,OTA_DONE)

                #wait for response 
                await asyncio.sleep(1)

                #disable notification for ota control characteristic
                await client.stop_notify(OTA_CONTROL_UUID)

                #check if successfully ended
                if await queue.get() == "ack":
                    print("ota update successful.........................")
                    end_time = datetime.datetime.now()
                    print("total update time:{}".format(end_time-begin_time))

                else:
                    print("ota update failed!!!!!!!!!!!!!!!!")


    except Exception as e:
        print(e)
    finally:
        pass
                
                


if __name__=='__main__':
    #select your path to binary file below
    asyncio.run(__ota_main__("C:\\Users\\chait\\esp\\v5.0.6\\esp-idf\\esp_projects\\ESP32-BLE-OTA_DS18B20\\BLE_OTA_ESP32_DS18B20\\build\\BLE_OTA_ESP32_DS18B20.bin"))