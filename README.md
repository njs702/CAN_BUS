# CAN 통신이란?

CAN 통신(Controller Area Network)은 차량 내에서 호스트 컴퓨터 없이 마이크로 컨트롤러나 장치들이 서로 통신하기 위해 설계된 표준 통신 규격이다. CAN 통신은 메시지 기반 프로토콜이며 최근에는 차량 뿐만 아니라 산업용 자동화기기나 의료용 장비에서도 종종 사용되고 있다. Controller Area Network (CAN)은 각 제어기들 간의 통신을 위해 주로 사용되는 non-host 버스 방식의 메시지 기반 네트워크 프로토콜이다.

---
* 1986년 Bosch사에 의해 공개되었다.
* 현재 생산되는 자동차에서 사용되고 있다.
* 우성 비트(dominant bit, 0)와 열성 비트(recessive bit,1)를 갖는다.

## 1. UART 통신과의 차이점?
UART 통신은 모듈이 추가될 때마다 연결선이 추가적으로 필요하다. 수 많은 연결선들은 비용의 증대와 공간의 비 효율성을 낳게 된다.

<p align="center"><img src="./img/ecu_parallel.PNG"></p>

CAN 인터페이스는 여러 개의 모듈을 처리할 수 있게 해주고, 선의 감소와 비용의 감소 공간의 효율성을 증대시켰다. CAN 통신은 여러 ECU를 병렬로 연결해 데이터를 주고받는다. CAN 버스를 통해 필요한 데이터에 접근할 수 있다.

## 2. CAN 통신의 특징

### 2.1 시리얼 통신 기반
> CAN 통신은 Serial Network 통신 방식의 일종이다. 여러 ECU를 병렬로 연결하여 서로 정보를 교환한다. 위의 그림을 보면 알 수 있듯이, 한 라인에 모든 노드가 연결되고 모든 노드는 같은 통신속도로 연결된다. 모든 노드는 같은 버스라인을 통해 데이터를 전송 및 수신한다.

### 2.2 Multi-Master
> 모든 CAN 구성 모듈은 정보 메시지 전송에 자유 권한이 있다. 즉, 통신 버스를 여러 노드들이 공유하면서 언제든지 버스를 사용할 수 있는 것이다.

### 2.3 간단하고 노이즈에 강하다
> CAN H, CAN L 두 개의 신호로 통신하기 때문에 선이 두개만 필요하다. 따라서 더 많은 모듈들이 추가되더라도 추가되는 선의 양이 적다는 이점이 있다. Twist Pair 구조로 되어 있기 때문에 노이즈에 강하다.

### 2.4 ID 값을 이용한 우선순위에 따라 처리한다
> 자동차의 ECU들은 고유한 ID 값을 가지고 있다. CAN 통신에서는 ID 값이 낮을수록 우선순위가 높은데, 여러 과정을 통해 설정된 ID 값을 수신하여 우선순위를 결정한다. 주소가 아니라 ID 값으로 메시지를 관리하기 때문에 시스템 제어 속도와 안전성을 향상할 수 있다. 낮은 우선순위의 메시지는 다음 bus cycle 때 재전송된다.

### 2.5 기타 CAN 통신의 특징
* CSMA(Carrier Sense Multiple Access)
* CD-CR(Collision Detection with Collision Resolution)
* 최대 8 BYTE 전송
* 주소 지정방식 X
* 비동기식 직렬 통신, 듀얼 와이어 접속 방식
* 고속 통신 및 신뢰성 / 안전성(에러 검출 및 처리 성능 우수)

## 3. CAN 통신의 장점
* 각각의 ECU들 간에 정보 교환이 이루어진다.
* 여러 장치를 2개의 선(Twisted Pair Wires)으로 컨트롤 할 수 있다.
* 통신이 되는 라인을 BUS-A(CAN H), BUS-B(CAN L)이라고 한다.
* 따라서 하나의 선에 에러가 나더라도 다른 선에 의해 정상적인 통신이 가능하다.

<p align="center"><img src="./img/can_module.png"></p>

CAN 통신은 전체 노드를 제어하는 Master가 없기 때문에 데이터에 쉽게 접근할 수 있다. ID값을 통해 불필요한 메시지를 무시하고 자신에게 필요한 메시지만 수신한다. 다중 노드가 동시에 버스에 메시지를 전송하려는 경우, ID값이 가장 낮은 정보를 가진 노드가 최 우선으로 버스에 접근하게 된다.

## 4. CAN Data Frame Format

<p align="center"><img src="./img/can_data_frame.png"></p>

* Arbitration Field : 메시지 ID가 있는 필드
* Control Field : 메시지 제어에 필요한 비트 값을 가지고 있는 필드, IDE 비트 값을 통해 프레임이 Standard Frame인지 Extended Frame인지 확인하고, DLC 비트를 통해 메시지 데이터의 길이를 알려준다
* Data Field : 실질적인 통신내용인 데이터가 있는 필드이다. CAN 메시지의 경우, 최대 8 Byte까지의 데이터를 메시지에 실을 수 있다
* Check Field : 변형된 데이터가 없는지 확인하는 CRC 값들이 있는 필드
* ACK Field : 메시지를 잘 수신했다는 것을 버스에 있는 다른 노드들에게 알려주는 값이 있는 필드

### 4.1 Standard CAN message format
메시지 ID, 식별자의 길이가 11 Bit인 형식이다. CAN 통신에서 메시지 ID는 메시지 고유의 이름일 뿐만 아니라, CAN 버스에서 2개 이상의 메시지 Collision이 일어날 경우 중재에 사용되는 우선순위의 역할도 가지고 있다. 과거 11 Bit로 충분한 크기의 범위였지만, 시간이 흐르고 메시지가 더 많이 필요해지게 되어 29 Bit의 메시지 ID를 갖고 있는 Extended Frame이 생기게 되었다.

<p align="center"><img src="./img/can_standard.PNG"></p>

### 4.2 Extended CAN message format

<p align="center"><img src="./img/can_extended.PNG"></p>

### 4.3 Remote CAN message format
Remote Frame은 데이터 필드를 가지고 있지 않은 프레임이다. 메시지를 제대로 수신하지 못해서 해당 메시지의 재전송이 필요한 경우 전송되는 프레임이다. 모든 데이터를 정확히 수신해야 하거나, 대용량의 Data가 송수신되는 비행기 혹은 선박 업계에서 사용된다.

### 4.4 Error CAN message format