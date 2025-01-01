import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import Int32

class Room():
    def __init__(self,area:int=0,prefTemp:int= 0,temp:int=0,tcs:int=0):
        self.area=area
        self.prefTemp=prefTemp
        self.temp=temp
        self.tcs=tcs

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        self.roomCount:int=0
        self.rooms:list[Room]=[None]
        self.roomCountSub=self.create_subscription(Int32,"/Temp/RoomCount",self.RoomCountCallBack,10)
        self.roomTempSubList:list = [None]
        self.roomPrefTempSubList:list = [None]
        self.roomAreaSubList:list = [None]
        self.roomtcsPubList:list = [None]
        self.printLog = self.create_timer(1.0,self.MainCallBack)

    def RoomCountCallBack(self, msg:Int32):
        self.roomCount = msg.data
        self.RoomAdd()
    def RoomAdd(self):
        self.rooms.append(Room())
        self.roomtcsPubList.append(self.create_publisher(Temperature, f"/Temp/Room{self.roomCount}Temp", 10))
        self.roomTempSubList.append(self.create_subscription(Temperature,f"/Temp/Room{self.roomCount}Temp",self.RoomTempCallBack,10))
        self.roomPrefTempSubList.append(self.create_subscription(Temperature,f"/Temp/Room{self.roomCount}PrefTemp",self.RoomPrefTempCallBack,10))
        self.roomAreaSubList.append(self.create_subscription(Temperature,f"/Temp/Room{self.roomCount}Area",self.RoomAreaCallBack,10))

    def RoomTempCallBack(self, msg:Temperature):
        self.rooms[msg.temperature] = msg.variance
    def RoomPrefTempCallBack(self, msg:Temperature):
        self.rooms[msg.temperature] = msg.variance
    def RoomAreaCallBack(self, msg:Temperature):
        self.rooms[msg.temperature] = msg.variance
    
    def MainCallBack(self):
        count = 0
        for roomtcsPub in self.roomtcsPubList:
            roomtcsMsg = Int32()
            if(self.rooms[count]==None):
                break
            roomtcsMsg.data = self.rooms[count].temp
            roomtcsPub.publish(roomtcsMsg)
        i=1
        roomslog:str = "\n\tRoomsInfo:"
        for room in self.rooms:
            if(room==None):
                roomslog+="    No room."
                break
            roomslog += f"\n-Room{i}:Area={room.area}m²/Temp={room.temp}°C/PreferredTemp={room.prefTemp}°C"
        self.get_logger().info(f"RoomsCount:{self.roomCount}"+roomslog)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
