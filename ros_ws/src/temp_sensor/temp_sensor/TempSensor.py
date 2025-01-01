#!/usr/bin/env python3
import rclpy
import random
from datetime import date,datetime
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import Int32
from std_srvs.srv import Empty
from example_interfaces.srv import AddTwoInts



class Room():
    def __init__(self,area:int=0,prefTemp:int= 0,temp:int=0,tcs:int=0):
        self.area=area
        self.prefTemp=prefTemp
        self.temp=temp
        self.tcs=tcs

class TempSensor(Node):
    def __init__(self):
        super().__init__("TempSensor")
        self.currentdate = date.today()
        self.hour = datetime.now().hour
        self.month = self.currentdate.month
        self.maxminTemp:list[int, int] = self.RandomOutTemp()
        self.outTemp:float
        self.tco:float
        self.tcs:float = 0
        self.InitOutTemp()
        self.InitOTC()
        self.rooms:list[Room] = [None]
        self.outTempPub = self.create_publisher(Temperature, "/Temp/OutTemp", 10)
        self.roomCountPub = self.create_publisher(Int32, "/Temp/RoomCount", 10)
        self.roomTempPubList:list = [None]
        self.roomAreaPubList:list = [None]
        self.UpdateMaxMinTemp = self.create_timer(86400.0,self.RandomOutTemp)
        self.UpdateOTC = self.create_timer(86400.0,self.InitOTC)
        self.printLog = self.create_timer(1.0,self.MainCallBack)

    def RandomOutTemp(self):
        if(self.month==12 or self.month==1 or self.month==2):
            return [random.randint(0, 5),random.randint(10, 18)]
        elif(self.month==6 or self.month==7 or self.month==8):
            return [random.randint(18, 23),random.randint(30, 40)]
        elif(self.month==3 or self.month==4 or self.month==5 or 
                    self.month==9 or self.month==10 or self.month==11):
            return [random.randint(10, 15),random.randint(20, 27)]
    def InitOutTemp(self):
        if(self.hour>=6 and self.hour<=16):
            self.outTemp = self.maxminTemp[0]+(self.maxminTemp[1]-self.maxminTemp[0])*(self.hour-6)/10
        elif(self.hour>=16 and self.hour<=24):
            self.outTemp = self.maxminTemp[1]-(self.maxminTemp[1]-self.maxminTemp[0])*(self.hour-16)/14
        elif(self.hour>=0 and self.hour<=16):
            self.outTemp = self.maxminTemp[0]+(self.maxminTemp[1]-self.maxminTemp[0])*(6-self.hour)/14
    def InitOTC(self):
        if(self.hour>=6 and self.hour<=16):
            self.tco = (self.maxminTemp[1]-self.maxminTemp[0])/10
        elif((self.hour>=16 and self.hour<=24) or (self.hour>=0 and self.hour<=16)):
            self.tco = (self.maxminTemp[1]-self.maxminTemp[0])/14
         
    def AddRoom(self,room:Room):
        self.rooms.append(room)
        count = self.GetRoomCount()
        self.roomTempPubList.append(self.create_publisher(Temperature, f"/Temp/Room{count}Temp", 10))
        self.roomAreaPubList.append(self.create_publisher(Int32, f"/Temp/Room{count}Area", 10))

    def GetRoomCount(self)->int:
        count = 0
        if self.rooms!=None:
            for room in self.rooms:
                if room==None:
                    break
                count += 1
        return count
    
    def MainCallBack(self):
        roomCountMsg = Int32()
        roomCountMsg.data = self.GetRoomCount()
        self.roomCountPub.publish(roomCountMsg)
        outTempMsg = Temperature()
        outTempMsg.temperature = self.outTemp
        self.outTempPub.publish(outTempMsg)
        count = 0
        for roomTempPub in self.roomTempPubList:
            roomTempMsg = Temperature()
            if(self.rooms[count]==None):
                break
            roomTempMsg.temperature = self.rooms[count].temp
            roomTempMsg.variance = self.rooms[count].prefTemp
            roomTempPub.publish(roomTempMsg)
        count = 0
        for roomAreaPub in self.roomAreaPubList:
            roomAreaMsg = Int32()
            if(self.rooms[count]==None):
                break
            roomAreaMsg.data = self.rooms[count].area
            roomAreaPub.publish(roomAreaMsg)
        roomslog:str = "\n\tRoomsInfo:"
        i:int = 1
        for room in self.rooms:
            if(room==None):
                roomslog+="    No room."
                break
            roomslog += f"\n-Room{i}:Area={room.area}m²/Temp={room.temp}°C/PreferredTemp={room.prefTemp}°C"
        self.get_logger().info(f"Temp outside is:{self.outTemp}     RoomsCount:{self.GetRoomCount()}"+roomslog)

    def CallAddRoom(self):
        client = self.create_client(Empty,"/Temp/AddRoom")

def main(args=None):
    rclpy.init(args=args)
    node = TempSensor()
    rclpy.spin(node)
    rclpy.shutsown()