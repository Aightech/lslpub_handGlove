#include <stdio.h>
#include <stdlib.h>
#ifdef WIN32
#include <windows.h>
#else
#include <termios.h>
#endif

#include <fstream>
#include <sstream>

#include <iostream>
#include <fcntl.h>
#include <unistd.h>

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
#ifdef WIN32
  HANDLE handler;
  COMSTAT status;
  DWORD errors;
#else
  int fd;
#endif
  
  bool connected;
} Arduino;

void init_conn(Arduino *a, char *portName)
{
#ifdef WIN32
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
#else
  a->fd =open(portName, O_RDWR);
  if(a->fd == -1)
    error("[ERROR] Couldn't open port ");


  /* set the other settings (in this case, 9600 8N1) */
  struct termios settings;
  int v = tcgetattr(a->fd, &settings);
  if(v!=0)
    error("could not read conf the serial port");

  cfsetispeed(&settings, B9600);
  cfsetospeed(&settings, B9600);
  
  settings.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity.
  settings.c_cflag &= ~CSTOPB;  // Clear stop field, only one stop bit used in communication.
  settings.c_cflag |= CS8;      // 8 bits per byte
  settings.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
  settings.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  
  settings.c_lflag = ~ICANON;   // not canonical mode (don't wait new line)
  settings.c_lflag &= ~ECHONL;  // Disable new-line echo
  settings.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP
  settings.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  settings.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  

  settings.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  settings.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  
  settings.c_cc[VMIN] = 0;  // Timeout from start of read
  settings.c_cc[VTIME] = 1;// 100 ms timeout.

  v = tcsetattr(a->fd, TCSANOW, &settings); // Apply the settings
  if(v!=0)
    error("could not conf the serial port");
  //tcflush(a->fd, TCOFLUSH);
  usleep(1000000);
  std::cout << "[INFO] Glove configured." <<std::endl;
  
  
#endif
    
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
#ifdef WIN32
  DWORD bytesRead;	  
  DWORD bytesSend;
#endif
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
  
#ifdef WIN32
  WriteFile(a->handler, (void*) &c, 1, &bytesSend, 0);
  ClearCommError(a->handler, &(a->errors), &(a->status));
  ReadFile(a->handler, (char*)hand_open, 2*10, &bytesRead, NULL);
#else
  write(a->fd, &c, 1);
  read(a->fd, (char*)hand_close, 2*10);
#endif
  std::cout << "                         ";
  for(int i =0; i<10;i++)
    std::cout << hand_open[i] << " " ;
  std::cout << std::endl;
	    

  std::cout << "                       - Close your hand at the maximum. Then press any key and <RET> ..." << std::flush;
  std::cin >> o;
#ifdef WIN32
  WriteFile(a->handler, (void*) &c, 1, &bytesSend, 0);
  ClearCommError(a->handler, &(a->errors), &(a->status));
  ReadFile(a->handler, (char*)hand_close, 2*10, &bytesRead, NULL);
#else
  write(a->fd, &c, 1);
  read(a->fd, (char*)hand_close, 2*10);
#endif
  
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
				      "config/conf.cfg",
				      "0"});
  
  get_arg(argc, argv, opt_flag, opt_label, opt_value);
  std::string stream_name = opt_value[0];
  std::string calibration_file = opt_value[1];
  int calibration_mode = std::stoi(opt_value[2]);  

  Arduino arduino;
  float param[2*15];
  
#ifdef WIN32
  char *portName = "\\\\.\\COM4";
#else
  char *portName = "/dev/ttyUSB0";
#endif
  init_conn(&arduino, portName);

  if(calibration_mode==1)
    calibration(&arduino, calibration_file, param);
  else
      get_cal(calibration_file, param);

#ifdef WIN32
  DWORD bytesRead;	  
  DWORD bytesSend;
#endif
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
#ifdef WIN32
	  WriteFile(arduino.handler, (void*) &c, 1, &bytesSend, 0);
	  ClearCommError(arduino.handler, &(arduino.errors), &(arduino.status));
	  ReadFile(arduino.handler, (char*)buffer, 2*nb_in, &bytesRead, NULL);
#else
	  int n = write(arduino.fd, "a", 1);
	  int counter = 0;
	  while(n && counter < 2*nb_in)
	    {
	      n = read(arduino.fd, ((char *)buffer)+counter, 2*nb_in);
	      if(n==-1)
		error("issue -1");
	      counter+=n;
	      
	    }
	  std::cout << "["<<counter  << "]\t" << std::flush;
#endif
	  
	  
	  sample[nb_ch-1]=buffer[nb_in-1];
	  for(int i =0; i<nb_ch-1;i++)
	    {
	      sample[i] = (buffer[2*(i/3) + ((i%3==0)?1:0)])*param[2*i] + param[2*i+1];
	      std::cout << i << ":" << std::fixed << std::setprecision(3)  << sample[i] << "| ";
	    }
	  
	  std::cout << sample[nb_ch-1] << "\xd" << std::flush;
	    
	  outlet.push_sample(sample);
#ifdef WIN32
	  Sleep(1);
#else
	  usleep(100);
#endif
	}
     
	
    }
  catch (std::exception& e)
    {
      std::cerr << "[ERROR] Got an exception: " << e.what() << std::endl;
    }
    return 0;

#ifdef WIN32
    CloseHandle(arduino.handler);
#else
    close(arduino.fd);
#endif
  
}
