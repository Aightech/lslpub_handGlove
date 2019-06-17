#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <fstream>
#include <sstream>

#include <iostream>
#include <tools.h>
#include <lsl_cpp.h>
#include <iomanip>      // std::setprecision


#define HEADER '$'
#define ENDCAR '#'
#define CMD_SAMPLING 0x0A

#define ARDUINO_WAIT_TIME 2000
#define MAX_DATA_LENGTH 255

typedef struct _arduino
{
  HANDLE handler;
  bool connected;
  COMSTAT status;
  DWORD errors;
} Arduino;

void init_conn(Arduino *a, char *portName)
{
  a->handler = CreateFileA(static_cast<LPCSTR>(portName),
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			NULL);
  if (a->handler == INVALID_HANDLE_VALUE)
    error("ERROR: Handle was not attached. Reason:  not available\n");
        
 
  DCB dcbSerialParameters = {0};
  if (!GetCommState(a->handler, &dcbSerialParameters))
    error("failed to get current serial parameters");
  
  dcbSerialParameters.BaudRate = CBR_9600;
  dcbSerialParameters.ByteSize = 8;
  dcbSerialParameters.StopBits = ONESTOPBIT;
  dcbSerialParameters.Parity = NOPARITY;
  dcbSerialParameters.fDtrControl = DTR_CONTROL_ENABLE;

 
  if(!SetCommState(a->handler, &dcbSerialParameters))
    error("ALERT: could not set Serial port parameters\n");
	  
	 
  PurgeComm(a->handler, PURGE_RXCLEAR | PURGE_TXCLEAR);
  Sleep(ARDUINO_WAIT_TIME);
    
}

void set_cal(std::string file, float param[])
{
  std::ofstream myfile (file,std::ios::trunc);
  if (myfile.is_open())
    for(int i =0; i< 2*15 ; i++)
      myfile << param[i] << " ";
  myfile.close();
}

void get_cal(std::string file, float param[])
{
  std::ifstream source;                    // build a read-Stream
  source.open(file, std::ios_base::in);  // open data
  if (source)
    {
      std::string line;
      std::getline(source, line);
      std::istringstream in(line);
      int a;
      for(int i =0; i< 2*15 ; i++)
	in >> param[i];
    }
  else
    {
      std::cout << "could not open the calibration file." << std::endl;
      for(int i =0; i< 2*15 ; i++)
	param[i]=(i+1)%2;
    }
}

void calibration(Arduino* a, std::string file, float param[])
{
  DWORD bytesRead;	  
  DWORD bytesSend;
  char c = 'a';
  char o;
  int16_t hand_open[10];
  int16_t hand_close[10];
  std::cout << std::endl;
  std::cout << "+-------------------+" << std::endl;
  std::cout << "| Calibration mode: |" << std::endl;
  std::cout << "+-------------------+" << std::endl;
  std::cout << "                       - Open your hand at the maximum. Then press any key and <RET> ..." << std::flush;
  std::cin >> o;
  WriteFile(a->handler, (void*) &c, 1, &bytesSend, 0);
  ClearCommError(a->handler, &(a->errors), &(a->status));
  ReadFile(a->handler, (char*)hand_open, 2*10, &bytesRead, NULL);
  std::cout << "                         ";
  for(int i =0; i<10;i++)
    std::cout << hand_open[i] << " " ;
  std::cout << std::endl;
	    

  std::cout << "                       - Close your hand at the maximum. Then press any key and <RET> ..." << std::flush;
  std::cin >> o;
  WriteFile(a->handler, (void*) &c, 1, &bytesSend, 0);
  ClearCommError(a->handler, &(a->errors), &(a->status));
  ReadFile(a->handler, (char*)hand_close, 2*10, &bytesRead, NULL);
  std::cout << "                         ";
  for(int i =0; i<10;i++)
    std::cout << hand_close[i] << " " ;
  std::cout << std::endl;

  for(int i =0; i<15; i++)
    {
      param[2*i] = (0-100)/(float)( hand_open[2*(i/3) + ((i%3==0)?1:0)] - hand_close[2*(i/3) + ((i%3==0)?1:0)]);
      param[2*i + 1] = - hand_close[2*(i/3) + ((i%3==0)?1:0)]*param[2*i] + 100;

      std::cout <<  param[2*i] << " " <<  param[2*i+1] << std::endl;
  
    }
  set_cal(file, param);
  
}


int main(int argc, char *argv[])
{
  std::vector<std::string> opt_flag(
				    {"-n",
				     "-c",
				    "-cal"});
  std::vector<std::string> opt_label(
				     {"Lsl output stream's name",
				      "Calibration file",
				      "Calibration Mode: 0:No / 1:Yes"});
  std::vector<std::string> opt_value(
				     {"handGlove",
				      "conf.cfg",
				      "0"});
  
  get_arg(argc, argv, opt_flag, opt_label, opt_value);
  std::string stream_name = opt_value[0];
  std::string calibration_file = opt_value[1];
  int calibration_mode = std::stoi(opt_value[2]);  

  Arduino arduino;
  float param[2*15];
  

  char *portName = "\\\\.\\COM4";
  init_conn(&arduino, portName);

  if(calibration_mode==1)
    calibration(&arduino, calibration_file, param);
  else
      get_cal(calibration_file, param);
  
  DWORD bytesRead;	  
  DWORD bytesSend;
  char c = 'a';

  int nb_in=11;
  int16_t buffer[nb_in];  
  
  int nb_ch = 16;
  std::vector<float> sample(nb_ch);
  try
    {
      //LSL stream output
      lsl::stream_info info(stream_name, "gloveSamples", nb_ch, 0, lsl::cf_float32);
      lsl::stream_outlet outlet(info);
      for(;;)
	{ 
	  
	  WriteFile(arduino.handler, (void*) &c, 1, &bytesSend, 0);
	  ClearCommError(arduino.handler, &(arduino.errors), &(arduino.status));
	  ReadFile(arduino.handler, (char*)buffer, 2*nb_in, &bytesRead, NULL);

	  
	  sample[nb_ch-1]=buffer[nb_in-1];
	  for(int i =0; i<nb_ch-1;i++)
	    {
	      sample[i] = (buffer[2*(i/3) + ((i%3==0)?1:0)])*param[2*i] + param[2*i+1];
	      std::cout << std::fixed << std::setprecision(3)  << sample[i] << " ";
	    }
	  
	  std::cout << sample[nb_ch-1] << "\xd" << std::flush;
	    
	  outlet.push_sample(sample);
	  Sleep(10);
	}
     
	
    }
  catch (std::exception& e)
    {
      std::cerr << "[ERROR] Got an exception: " << e.what() << std::endl;
    }
    return 0;
    CloseHandle(arduino.handler);
  
}
