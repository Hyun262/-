import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
GPIO.setwarnings(False)

scale = [261, 294, 329, 349, 392, 440, 493, 523]
list = [2,1,2,1]
list_1 = [5,4,5,4]
list_2 = [7,7,7,7]
term = [0.5,0.5,0.5,0.5]

# MQTT 설정
mqtt_broker = "broker.emqx.io"
mqtt_topic = "mobile/lhk/sensing"

# GPIO 핀 번호 설정
yellow_led_pin = 23
red_led_pin = 24
buzzer_pin = 12
trig_pin = 13
echo_pin = 19

# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)

# GPIO 핀 설정
GPIO.setup(yellow_led_pin, GPIO.OUT)
GPIO.setup(red_led_pin, GPIO.OUT)
GPIO.setup(buzzer_pin, GPIO.OUT)
GPIO.setup(trig_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)

# 초음파센서 함수
def measure_distance():
    # 초음파 발신 시간 초기화
    GPIO.output(trig_pin, False)
    time.sleep(0.5)
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    # 초음파 수신 시간 측정
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()

    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()

    # 거리 계산 
    distance = (pulse_end - pulse_start) * 17000
    distance = round(distance, 2)
    print("Distance : ", distance, "cm")

    return distance

# MQTT 메시지 전송 함수
def send_mqtt_message(message):
    client = mqtt.Client()
    client.connect(mqtt_broker, 1883, 60)
    client.publish(mqtt_topic, message)
    client.disconnect()

try:
    while True:
        distance = measure_distance()

        if distance <= 40 and distance >= 20:
            GPIO.output(yellow_led_pin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(yellow_led_pin, GPIO.LOW)
            time.sleep(0.5)

            p = GPIO.PWM(buzzer_pin, 100)
            p.start(100)
            p.ChangeDutyCycle(90)
    
            for i in range(4):
                p.ChangeFrequency(scale[list[i]])
                time.sleep(term[i])
        
            p.stop()

        elif distance < 20 and distance >= 5:
            GPIO.output(red_led_pin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(red_led_pin, GPIO.LOW)
            time.sleep(0.5)
            p = GPIO.PWM(buzzer_pin, 100)
            p.start(100)
            p.ChangeDutyCycle(90)
    
            for i in range(4):
                p.ChangeFrequency(scale[list_1[i]])
                time.sleep(term[i])
        
            p.stop()

        elif distance < 5:
            GPIO.output(red_led_pin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(red_led_pin, GPIO.LOW)
            time.sleep(0.5)
            p = GPIO.PWM(buzzer_pin, 100)
            p.start(100)
            p.ChangeDutyCycle(90)
    
            for i in range(4):
                p.ChangeFrequency(scale[list_2[i]])
                time.sleep(term[i])
        
            p.stop()
            send_mqtt_message("사고가 발생했습니다")

        else:
            GPIO.output(yellow_led_pin, GPIO.LOW)
            GPIO.output(red_led_pin, GPIO.LOW)
            GPIO.output(buzzer_pin, GPIO.LOW)

except KeyboardInterrupt:
    GPIO.cleanup()


