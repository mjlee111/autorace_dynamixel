#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include <queue>
#include <mutex>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace serial;

void serialReadThread();
void run();
void syncwrite2(uint8_t id1, uint16_t pos);
vector<uint8_t> packet_read;

int cnt = 0;

Serial s;
mutex m;
bool serialRead = false;
queue<uint8_t> readByte;

int main(int argc, char **argv)
{
  // ros init
  ros::init(argc, argv, "comm");
  ros::start();
  ros::NodeHandle n;
  // 포트, 보드레이트 설정
  s.setPort("/dev/ttyUSB1");
  s.setBaudrate(1000000);

  // 포트 존재하지 않거나 할 경우 대비한 예외처리
  try
  {
    s.open();
  }
  catch (serial::IOException e)   // 퍼미션, 포트 없음 등으로 열수 없을 때 예외 출력하고 리턴
  {
    cerr << "Port open failed." <<e.what()<<endl;
    return false;
  }

  cout<<"Dynamixel port opened"<<endl;
  // 수신 루프 정지를 위한 변수. true이면 계속 수신하다가 false가 되면 루프 종료
  serialRead = true;
  thread readThread(serialReadThread);
  ros::Rate loopRate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    if(cnt % 2 == 0) syncwrite2(6, 1023);
    else syncwrite2(6, 0);
    cnt++;
    loopRate.sleep();
  }
  serialRead = false; // 스레드 join울 위해 false로
  readThread.join();  // 수신 루프 종료 기다려 join
  return 0;
}

// 수신 확인, 수신을 위한 스레드
void serialReadThread()
{
  // 수신한 바이트를 저장할 버퍼
  unsigned char readBuffer[100];

  // 수신 루프
  while(serialRead)
  {
    try // 중간에 통신선이 뽑히거나 할 경우 대비해 예외처리
    {
      int size = s.available();
      if (size != 0)  // 받은 바이트 수가 0이 아닐때
      {
        s.read(readBuffer, size);   // 버퍼에 읽어옴

        m.lock();   // producer consumer pattern. 뮤텍스 lock
        for(int i=0; i<size; i++) readByte.push(readBuffer[i]);    // readByte에 하나씩 저장.
        m.unlock(); // 뮤텍스 unlock
      }
    }
    catch (IOException e)   // 예외 발생시 메시지 띄우고 포트 닫는다.
    {
      cerr << "Port disconnected. closing port(" << e.what() << ")." << endl;
      s.close();
    }

    this_thread::sleep_for(chrono::milliseconds(1));    // cpu 점유율 낮추기 위해 잠깐 sleep
  }
}

void syncwrite2(uint8_t id1, uint16_t pos){
  uint8_t pos_L, pos_H, spd_L, spd_H;
  uint16_t spd = 1023;
  pos_L = (pos & 255);
  pos_H = ((pos >> 8) & 255);
  spd_L = (spd & 255);
  spd_H = ((spd >> 8) & 255);
  vector<uint8_t> packet;
  packet.push_back(0xFF);
  packet.push_back(0xFF);
  packet.push_back(id1);
  packet.push_back(0x07);
  packet.push_back(0x03);
  packet.push_back(0x1E);
  packet.push_back(pos_L);
  packet.push_back(pos_H);
  packet.push_back(spd_L);
  packet.push_back(spd_H);
  uint8_t cks = 0;
  for(int i = 2 ; i < 10 ; i++) cks += packet[i];
  cks = ~cks;
  packet.push_back(cks);
  s.write(packet);
  if(cnt % 2 == 0) cout << "out1" << endl << endl;
  else cout << "out2" << endl << endl;
}

