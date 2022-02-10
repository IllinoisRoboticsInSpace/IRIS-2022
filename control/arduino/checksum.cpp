#include <iostream>
typedef __UINT8_TYPE__ byte;
using std::cout;
using std::endl;


int largestOneIndex(unsigned int curr) {
  int i = 15;
  while (i >= 0) {
    if (curr & (1 << i)) {
      break;
    }
    i--;
  }
  return i;
}
byte checksum(byte OdroidIn[2]) //generates first byte, creating its checksum and adding it to the motor number
{
   unsigned int OdroidInInt = OdroidIn[1]+OdroidIn[0]*256;
   unsigned int CRC = 18;
   unsigned int msg = OdroidInInt % 2048;
   unsigned int checksum = OdroidInInt / 2048;
   unsigned int curr = msg*32 + checksum;

   int a, index;
   while (curr >= 32) {
    index = largestOneIndex(curr);
    a = CRC << (index - 4);
    curr ^= a;
   }
   if (curr) {
    // CRC came back bad
    // do some error handling here??

    cout << curr << endl;
    cout << curr*8 + OdroidIn[0] << endl;
    return byte(curr*8 + OdroidIn[0]);
   }
   
  //First byte: contains motorVal and checksum, in the Order Checksum:[4:0],motorVal[2:0]
  //Second byte: is the power value.
  /*
   * Input[0],[1],[2] correspond to motor
  KEY:
  Motor ID: 
0: Left Drive
1: Right Drive
2: Bucket Ladder M
3: dump conveyor LA
4: bucket ladder LA
5: agitator
6: dump conveyor M
7: stops motors
  */
  return 0;
  }
  int main(){
      byte OdroidIn[2] = {2, 30};
      OdroidIn[0] = checksum(OdroidIn);
      cout << "In[0] = " << int(OdroidIn[0]) << ", In[1] = " << int(OdroidIn[1]) << endl;
  }