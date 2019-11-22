# S1, S2, S3... correspondem aos sensores da esquerda para a direita do ponto 
# de vista do carrinho.

""" Sensores da esquerda:
        Se o erro eh negativo, então o carrinho está indo para a direita e o 
        carrinho deve curvar para a esquerda.
        Se o erro é positivo, então o carrinho está indo para a esquerda.
    
    Sensores da direita:
        Se o erro eh negativo, então o carrinho está indo para a esquerda e o 
        carrinho deve curvar para a direita.
        Se o erro é positivo, então o carrinho está indo para a esquerda.

    Resumo:
        Erro negativo significa: va para a esquerda
        Erro positivo significa: va para a direita
"""

import busio
import time
import RPi.GPIO as GPIO
import timeit
GPIO.setmode(GPIO.BCM)

class Motor:
	def __init__(self, pin1, pin2, en):

		self.__pin1 = pin1
		self.__pin2 = pin2
		self.__en = en

		GPIO.setup(pin1, GPIO.OUT) # frente
		GPIO.setup(pin2, GPIO.OUT) # atrás
		GPIO.setup(en, GPIO.OUT) # Motor direito enable

	def setPWM(self, DutyCycle, freq):
		p = GPIO.PWM(self.__en, freq) # Sets PWM pin
		
		p.start(DutyCycle) # Sets DutyCyle
		return p

	def updateMotorSpeed(self, p, newDC):
		p.ChangeDutyCycle(newDC)

	def stop(self):
		GPIO.output(self.__pin1, True) # frente
		GPIO.output(self.__pin2, True) # atrás

	def goForward(self):
		GPIO.output(self.__pin1, True) # frente
		GPIO.output(self.__pin2, False) # atrás
        
    def goBackward(self):
        GPIO.output(self.__pin1, False) # frente
        GPIO.output(self.__pin2, True) # atrás
        
class Sensors:
	def __init__(self,pin1,pin2,pin3,pin4,pin5,pin6):

		self.__pin1 = pin1
		self.__pin2 = pin2
		self.__pin3 = pin3
		self.__pin4 = pin4
		self.__pin5 = pin5
		self.__pin5 = pin6

        # Os sensores estão ordenados da esqueda para a direita olhando de trás
        # do carrinho
		GPIO.setup(pin1, GPIO.IN) # Sensor 1
		GPIO.setup(pin2, GPIO.IN) # Sensor 2
		GPIO.setup(pin3, GPIO.IN) # Sensor 3
		GPIO.setup(pin4, GPIO.IN) # Sensor 4
		GPIO.setup(pin5, GPIO.IN) # Sensor 5
		GPIO.setup(pin6, GPIO.IN) # Sensor 6

		self.__signals = []
		self.__signals.append(GPIO.input(pin1))
		self.__signals.append(GPIO.input(pin2))
		self.__signals.append(GPIO.input(pin3))
		self.__signals.append(GPIO.input(pin4))
		self.__signals.append(GPIO.input(pin5))
		self.__signals.append(GPIO.input(pin6))

	def getLuminousSignal(self):

		self.__signals[0] = (GPIO.input(self.__pin1))
		self.__signals[1] = (GPIO.input(self.__pin2))
		self.__signals[2] = (GPIO.input(self.__pin3))
		self.__signals[3] = (GPIO.input(self.__pin4))
		self.__signals[4] = (GPIO.input(self.__pin5))
		self.__signals[5] = (GPIO.input(self.__pin6))

		return self.__signals
    
class Tacometer:
	def __init__(self, pin):

		self.__count = 0;
		GPIO.setup(pin,GPIO.IN)

        GPIO.add_event_detect(pin, GPIO.RISING) 

	def getTacoCount(self):
        begin = time.time()
    	end = time.time()
    	self.__count = 0
    	while ((end-begin)<0.2):
    		if (GPIO.event_detected(7)):
    			self.__count = self.__count + 1
    		end = time.time()
        speed = (self.__count*(60/0.2)/20);
    	print("RPM = ", speed) #20 e o numero de furos no encoder

		return speed
    
class Controller:
	def __init__(self, leftPinFront, leftPinBack, leftEnable, rightPinFront, rightPinBack, rightEnable, Kp, Ki, Kd, deltaT, DutyCDMax, DutyCEMax, sensor1, sensor2, sensor3, sensor4, sensor5, sensor6):

        # Initialize sensors
        self.__sensors = Sensors(sensor1, sensor2, sensor3, sensor4, sensor5, sensor6);
        
        # Initialize tacometer
        self.__tacometer = Tacometer(7);
        # Initialize motor
        self.__leftMotor = Motor(leftPinFront,leftPinBack, leftEnable);
		self.__rightMotor = Motor(rightPinFront, rightPinBack, rightEnable);
		
        # SetPWM
        self.__p1 = self.__rightMotor.setPWM(100, 1000);
        self.__p2 = self.__leftMotor.setPWM(100, 1000);
        
        # Indicate foward movement to start the application
        self.__leftMotor.goForward();
        self.__rightMotor.goForward();
        
        self.__previousError = 0;
		self.__currentError = 0;
		self.__previousCommand = 0;
		self.__SetPoint = 2.5;
		self.__command = 0;

		self.__Kp = Kp
		self.__Ki = Ki
		self.__Kd = Kd
        
        # DutyCycles
        self.__DutyCDMax = 100;
        self.__DutyCEMax = 100;
        self.__N = 6;
        self.__lapNumber = 2;    
            
        self.__counter = 0;
        self.__h1 = deltaT/2;
        self.__h2 = 1/deltaT;
        self.__b0 = Kp + Ki*h1 + Kd*h2;
        self.__b1 = -Kp + Ki*h1 + Kd*h2;
        self.__b2 = Kd*h2;
        self.__a1 = -1;
        
        self.__lapCounter=0;

    def getError(self, x, SP):
    	soma = 0;
    	for i in range(len(x)):
    		if x[i] == 1:
    			soma += i;
    
    	if x.count(1) == 0:
    		errorPos = self.__previousError;
    
    	else:
    		linePos = soma/x.count(1);
    		errorPos = (linePos - SP)
    	return errorPos;

	def calculateTrajectory(self):
		S = self.__sensors.getLuminousSignal(); #Sensors signals
    
		self.__prePreviousError = self.__previousError;

		self.__previousError = self.__currentError;

		self.__currentError = getError(S, self.__SetPoint);

		self.__previousCommand = self.__command;

		self.__command = self.__b0*self.__currentError + self.__b1*self.__previousError + self.__b2*self.__prePreviousError - self.__a1*self.__previousCommand;

# The ideal would be to have command varying between 0 and 1
		if self.__command > 1:
			self.__command = 1
		elif self.__command < -1:
			self.__command = -1

		if sum(S) > 3:
			self.__rightMotor.updateMotorSpeed(self.__p1, 50);
			self.__leftMotor.updateMotorSpeed(self.__p2, 50);

		elif self.__command < 0: # Go to the left
			self.__rightMotor.updateMotorSpeed(self.__p1, DutyCDMax); # Motor direito
			self.__leftMotor.updateMotorSpeed(self.__p2, (DutyCEMax + command * DutyCEMax)); # Motor esquerdo

		elif self.__command > 0: # Go to the right
            self.__rightMotor.updateMotorSpeed(self.__p1, (DutyCDMax - command * DutyCDMax)); # Motor direito
			self.__leftMotor.updateMotorSpeed(self.__p2, DutyCEMax); # Motor esquerdo

		elif self.__command == 0:
            self.__rightMotor.updateMotorSpeed(self.__p1, 0.8*DutyCDMax); # Motor direito
			self.__leftMotor.updateMotorSpeed(self.__p2, 0.8*DutyCEMax); # Motor esquerdo

# Stop
		if sum(S) == self.__N: # Carrinho para no final supondo alta luminosidade
			self.__counter += 1;
			if self.__counter == 9:
				self.__lapCounter += 1;
				print(self.__lapCounter)
				if self.__lapCounter == self.__lapNumber:
					self.__leftMotor.stop();
                    self.__rightMotor.stop();
		else:
			self.__counter = 0;

		return;

# =============================================================================
# Main
# =============================================================================
if __name__ == "__main__":

    # Variables
    leftPinFront = 4;
    leftPinBack = 3;
    leftEnable = 13;
    rightPinFront = 27;
    rightPinBack = 22;
    rightEnable = 12;
    Kp = 50 #Kp = 50; # 5
    Ki = 0;
    Kd = 0.0005 #Kd = 0.0005; # 0.0009
    deltaT = 0.010;
    DutyCDMax = 100;
    DutyCEMax = 100;
    sensor1 = 14;
    sensor2 = 15;
    sensor3 = 18;
    sensor4 = 23;
    sensor5 = 24;
    sensor6 = 25;
    
    controller = Controller(leftPinFront, leftPinBack, leftEnable, rightPinFront, rightPinBack, rightEnable, Kp, Ki, Kd, deltaT, DutyCDMax, DutyCEMax, sensor1, sensor2, sensor3, sensor4, sensor5, sensor6);
    
    tic=timeit.default_timer();
    try:
    	while True:
    		tic=timeit.default_timer();
    
    		controller.calculateTrajectory();
    
    		while timeit.default_timer() < (tic + deltaT):
    			pass
    		tic=timeit.default_timer()
    finally:
    	GPIO.cleanup()
