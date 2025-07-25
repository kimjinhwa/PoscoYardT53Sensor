### POSCO Battery Management System
* modbus PC03 15, 16을 추가한다.
### 100KVA용 BMS 설치 자재 
* Main Bms : UPS에 부착하여 출고
* 센서BMS : 19개 
    - 센서간 연결 UPT Cable : 19개 
    - 2:1 UTP Jointer 1개
        UTP : 7,8 라인은 
    - 축전지 단자러그 산출 : 모듈에 설치된 배터리수 +1   
      ex) 290셀은 16셀용 17모듈, 9셀용 2모듈   
            17* 17 + 2*10  = 309개   
* 전류센서 모듈 : CS600E2/4V
* 전류센서 연결용 케이블(3M)

## 설치 방법. 
* UTP Jointer 사용
    - 모듈에 연결되는 라인은 가장 가까운 곳에 연결한다.
    - 배터리 함에서는 전원라인과 통신라인 두개가 결합되어 1개의 라인으로 센서모듈에 연결된다.
    - 전원라인 +15V: 1,2,4(오랜지띠, 오렌지, 청색) : +15V의 단자대에 연결
    - 전원라인 GND : 3,5,6(녹띠,녹,청띠) : 단자대의 -에  연결
    - RS485라인 : UPS에서 연결된 UTP라인을 체결  

      ![Jointer](jointer.jpg)
* 전류센서는 1번 장비에 연결한다.    
* 설치시 전류센서의 설치
  + 라인에 연결할 경우 사진과 같이 CT의 커넥터가 장비 방향으로 향하게 연결하며   
  - 라인에 연결할 경우 CT의 커넥터가 축전지 방향으로 향하게 설치한다. 
      ![CT설치그림](HOLECT_direction.png)
* 모듈의 설치는 1개의 모듈에는  6~16개를 설치하며   
  16셀의 경우 1번 핀은 -로 시작 하며, 2번 + ,,,, 16+로 총 17개의 라인이 연결된다. 
* LED가 계속 켜져 있으면, Short이거나 open-wire상태일 수 있다.
## 기능 추가 
- web 화면에서 각 모듈의 설정값을 변경할 수 있다. 
- modbus brocasting을 추가 하여 한번에 모든 단말의 06을 수행한다.
![설정그림](FRMT02_FUNCTIONSET1.png)
## 2025.06.25 
## 프로그램하면서 문제가 있던 사항. 
- H/W ESP32 CPU의 MISO Port를 반드시 Pull Down해 줘야 한다.   
  부팅에 문제가 있을 수 있다. 
- 온도 옵셋과 전류 옵셋을 추가한다.
- REF 4096
- TotalVoltage GAIN 1204 
- Ampere GAIN 1204 
- AmpereGain 1220 은 1.0이란 의미..1.22가 통상 맞는다. 
- Voltage Gain 1220
- 이것은 최종 결과 값에 대한 Gain을 곱한다.
- 새로운 문제가 나왔다. 양산한 보드가 작동을 하지 않는다.. 아마도 11163의 가능성이 높다.    
- 문제를 찾았다. 저항이 빠져 있었다.. OPAMP쪽에...
- 특이하게도 칩의 설정을 건드러면 데이타를 읽을 때 두번을 읽어야 적용 된다.    
   --> 이 부분은 해결을 하였다. 두번을 읽어야 하는 것이 맞다. 칩의 오류인것 같다.

## H/W설계 및 데이타 전송
### 데이타 전송
    * 총전압 *100으로 전송된다.
    - data[19] 총전압
    * 셀전압 *1000으로 전송된다.
    - data[0]~15 셀전압
    * 온도 T1, T2
      전송되는 값은 x100을 해서 정수형으로 전달
    - data[16], data[17] 온도 T1, T2
    * 전류 *100으로 전송된다.
    - data[18] 전류

## Sensor Board  
### 소비전력 측정
* 44mA 
    - 485 ON 상태
    - 대기전력
    - WiFi 전력
    -Max1161 구동시 전력 
    - WiFi + Max1161 + RS485 구동시 전력 
