
import RPi.GPIO as GPIO
import can
import time
import os

class TmotorCAN:
	def __init__(self, CANID) -> None:
		self.led = 22
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.led,GPIO.OUT)
		GPIO.output(self.led,True)
		
		self.pos = 0
		self.spe = 0
		self.torque = 0
		self.temp = 0
		
		self.pos_d=0
		self.v_des=0
		self.kp=0
		self.kd=0
		self.t_ff=0
		self.P_MIN=-12.5
		self.P_MAX=12.5
		self.V_MIN=-50
		self.V_MAX=50
		self.Kp_MIN=0
		self.Kp_MAX=500
		self.Kd_MIN=0
		self.Kd_MAX=5
		self.T_MIN=-54
		self.T_MAX=54
		
		self.p_int = 0xffff
		self.v_int = 0x0000
		self.kp_int = 0x0000
		self.kd_int = 0x0000
		self.t_int = 0x0000
		
		print('\n\rCAN Rx test')
		os.system("sudo /sbin/ip link set can0 down")
		time.sleep(0.1)	
		print('Bring up CAN0....')
		os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")
		time.sleep(0.1)	
		print('Press CTL-C to exit')
		try:
			self.bus = can.interface.Bus(channel='can0', interface="socketcan")
			print('Setup PiCAN board.')
		except OSError:
			print('Cannot find PiCAN board.')
			GPIO.output(self.led,False)
			exit()
	
		self.id = CANID
		self.data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
		print('CAN ready')
		self.message = self.bus.recv(1.0)
		print('CAN Open Success')
	
	def Motor_run(self):
		self.data=[0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc]
		self.msg = can.Message(arbitration_id=self.id,data=self.data, is_extended_id=False)
		self.bus.send(self.msg)
		print("motor " + str(self.id) + " RUNNING")
        
	def Motor_stop(self):
		self.data=[0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd]
		self.msg = can.Message(arbitration_id=self.id,data=self.data, is_extended_id=False)
		self.bus.send(self.msg)
		print("motor " + str(self.id) + " STOP")
        
	def set_origin(self):
		self.data=[0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe]
		self.msg = can.Message(arbitration_id=self.id,data=self.data, is_extended_id=False)
		self.bus.send(self.msg)
		print("motor " + str(self.id) + " New origin")
             

	def send_cmd(self, p_des, v_des, kp, kd, t_ff):
                
		self.p_int = self.ToUint(p_des,self.P_MIN,self.P_MAX,16)
		self.v_int = self.ToUint(v_des,self.V_MIN,self.V_MAX,12)
		self.kp_int = self.ToUint(kp,self.Kp_MIN,self.Kp_MAX,12)
		self.kd_int = self.ToUint(kd,self.Kd_MIN,self.Kd_MAX,12)
		self.t_int = self.ToUint(t_ff,self.T_MIN,self.T_MAX,12)
            
		self.data[0] = self.p_int>>8
		self.data[1] = self.p_int&0xFF
		self.data[2] = self.v_int>>4                    
		self.data[3] = ((self.v_int&0xF)<<4)|(self.kp_int>>8)
		self.data[4] = self.kp_int&0xFF
		self.data[5] = self.kd_int>>4
		self.data[6] = ((self.kd_int&0xF)<<4)|(self.t_int>>8)
		self.data[7] = self.t_int&0xff
            
		self.msg = can.Message(arbitration_id=self.id,data=self.data, is_extended_id=False)
		self.bus.send(self.msg)
		
	def ToUint(self, x, x_min, x_max, nbits):
		span = x_max - x_min

		if (x < x_min):
		    x = x_min

		if (x > x_max):
		    x = x_max
		toUint=((x - x_min) * ((float)((1 << nbits) - 1) / span))
		return int(toUint)
	
	def ToFloat(self,x_int, x_min, x_max, nbits):
		span = x_max - x_min
		offset_value = x_min
		toFloat= x_int * span / float((((1 << nbits) - 1))) + offset_value
		return toFloat
		
	def pos_control(self, p_des, kp, kd):
		self.pos_d=p_des*3.1416/180
		self.v_des=0
		self.kp=kp
		self.kd=kd
		self.t_ff=0
		self.send_cmd(self.pos_d, self.v_des, self.kp, self.kd, self.t_ff)
		
	def torque_control(self, torque_des):
		self.pos_d=0
		self.v_des=0
		self.kp=0
		self.kd=0
		self.t_ff=torque_des
		self.send_cmd(self.pos_d, self.v_des, self.kp, self.kd, self.t_ff)
		
	def decode(self):
	
		self.message = self.bus.recv()

		if self.message.data[0]==self.id:
			
			p_int = (self.message.data[1] << 8) | self.message.data[2]
			v_int = (self.message.data[3] << 4) | (self.message.data[4] >> 4)
			i_int = ((self.message.data[4] & 0xF) << 8) | self.message.data[5]
			temp_int = self.message.data[6]
			
			p = self.ToFloat(p_int, self.P_MIN, self.P_MAX, 16)
			v = self.ToFloat(v_int, self.V_MIN, self.V_MAX, 12)
			i = self.ToFloat(i_int, -self.T_MAX, self.T_MAX, 12)
			#Temp = self.ToFloat(temp_int, Temp_MIN, Temp_MAX, 8)
			
			self.pos = p
			self.spe = v
			self.torque = i
			#self.temp = Temp
			
			#print("yes"+ str(self.pos))
			
'''			
		c = '{0:f} {1:x} {2:x} '.format(self.message.timestamp, self.message.arbitration_id, self.message.dlc)
		s=''
		for i in range(self.message.dlc ):
			s +=  '{0:x} '.format(self.message.data[i])
			
		print(' {}'.format(c+s)+str(self.message.dlc))
'''
	
