#include <fcntl.h>
#include <unistd.h>
#include <iostream>

#include <vector>
#include <lsl_cpp.h>

#define HEADER '$'
#define ENDCAR '#'
#define CMD_SAMPLING 0x0A


#include <termios.h>



void error(const char *msg)
{
  printf("%s\n",msg);
  exit(0);
}

void usage(std::vector<std::string>& optf,  std::vector<std::string>& optl, std::vector<std::string>& optv)
{
  std::cout << "Usage: ./lslpub_OTB [OPTION ...]" << std::endl;
  std::cout << "Options: " << std::endl;
  for(int i = 0; i< optf.size(); i++)
    std::cout << "         " << optf[i] << "\t" << optl[i] <<" (ex: " << optv[i] << " )"<< std::endl;
  exit(0);
}

void get_arg(int argc, char ** argv, std::vector<std::string>& optf,  std::vector<std::string>& optl, std::vector<std::string>& optv)
{
  int i =1;
  std::string help_flag = "-h";
  while(i < argc)
    {
      if(help_flag.compare(argv[i])==0)
	usage(optf,optl,optv);
      int j = 0;
      while(optf[j].compare(argv[i]) != 0)
	{
	  j++;
	  if(j>= optf.size())
	    usage(optf,optl,optv);
	}
      if(i+1 >= argc || argv[i+1][0] == '-')
	usage(optf,optl,optv);
      optv[j] = argv[i+1];
      i+=2; 
    }
  for(int i = 0; i< optl.size(); i++)
    std::cout << optl[i] <<" : " << optv[i] << std::endl;
}

uint8_t crc(uint8_t *arr, int n)
{
  uint8_t crc=0;
  for(int i = 0; i<n; i++)
    {
      crc = (crc+arr[i])%0xFF;
    }
  return crc;
}

void config_serial(int fd)
{
  speed_t baud = B230400; /* baud rate */

  /* set the other settings (in this case, 9600 8N1) */
  struct termios settings;
  tcgetattr(fd, &settings);

  cfsetospeed(&settings, baud); /* baud rate */
  settings.c_cflag &= ~PARENB; /* no parity */
  settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
  settings.c_cflag &= ~CSIZE;
  settings.c_cflag |= CS8 | CLOCAL; /* 8 bits */
  settings.c_lflag = ICANON; /* canonical mode */
  settings.c_oflag &= ~OPOST; /* raw output */

  tcsetattr(fd, TCSANOW, &settings); /* apply the settings */
  tcflush(fd, TCOFLUSH);
  std::cout << "[INFO] Glove configured." <<std::endl;
  
}

void start_sampling(int fd)
{
  unsigned char command[6];
  command[0] = HEADER; // HEADER char
  command[1] = CMD_SAMPLING; // start sampling command
  command[2] = 0x03; // lengtht of the package
  command[3] = 0x01; //type of package
  command[4] = crc(command, 4); //sum of the previous bytes
  command[5] = ENDCAR; // ending char

  // // command[0] = HEADER; // HEADER char
  // // command[1] = 0x0B; // start sampling command
  // // command[2] = 0x02; // lengtht of the package
  // // command[3] = crc(command, 3); //sum of the previous bytes
  // // command[4] = ENDCAR; // ending char

  // //send a stop to be ensure a good start 
  // command[3] = 0x00;
  // command[4] = crc(command, 4);
  // write(fd, command, 5);
  // for(int i =0; i <6; i++)
  //   std::cout << (int) command[i] << std::endl;
  
  // usleep(1000000);

  // start sampling
  // command[3] = 0x01;
  // command[4] = crc(command, 4);

  
  if( write(fd, command, 6))
    std::cout << "[INFO] Glove started sampling" <<std::endl;
      
  
}

int main(int argc, char *argv[])
{
  std::vector<std::string> opt_flag(
				    {"-n"});
  std::vector<std::string> opt_label(
				     {"Lsl output stream's name"});
  std::vector<std::string> opt_value(
				     {"handGlove"});
  get_arg(argc, argv, opt_flag, opt_label, opt_value);
  
  std::string stream_name = opt_value[0];


  
  int glv_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
  if(glv_fd == -1)
    error("[ERROR] Couldn't ");
  std::cout << "[INFO] Glove port opened." <<std::endl;

  config_serial(glv_fd);
  start_sampling(glv_fd);
  

  int len, s, ind;
  unsigned char buffer[300];
  uint16_t id ;
  uint32_t clk ;
  int32_t quat[8];
  uint16_t finger[10];
  int nb_ch = 10;
  std::vector<float> sample(nb_ch);
  try
    {
      //LSL stream output
      lsl::stream_info info(stream_name, "gloveSamples", nb_ch, 0, lsl::cf_float32);
      lsl::stream_outlet outlet(info);
      for(;;)
	{
	  buffer[0] = 0;
	  while(buffer[0]!=HEADER)//search for the start of the reply
	    read(glv_fd, buffer, 1);
	  
	  
	  //read the command
	  read(glv_fd, buffer+1, 1);
	  //read the lenght
	  read(glv_fd, buffer+2, 1);
	  len = (int)buffer[2]-1;
	  //std::cout<< len << std::endl;
	  
	  if(len==88)
	    {
	      //get the data
	      s=0;
	      unsigned char* data= buffer+3;
	      while(s < len)
		s += read(glv_fd, data+s, len-s);
  	   
	      //if the last char is the ending char to ensure the datat is not too corrupted
	      if(data[len-1] == ENDCAR )
		{
		  ind = 1;//index of the data array
		  //read the id
		  id = __builtin_bswap16(((uint16_t*)((char*)data+ind))[0]);
		  ind+=2;

		  //read the clock
		  clk = __builtin_bswap32(((uint32_t*)((char*)data+ind))[0]);
		  ind+=4;

		  //read the quat position
		  for(int i =0;i< 8;i++, ind+=4)
		    quat[i] = __builtin_bswap32(((int32_t*)((char*)data+ind))[0]);

		  //read the finger flexxion
		  for(int i =0;i< 10;i++, ind+=2)
		    finger[i] = __builtin_bswap16(((int32_t*)((char*)data+ind))[0]);
	      
		  for(int i =0;i< 15;i++, ind+=2)
		    {
		      finger[i]=(finger[i]>1000)?1000:finger[i];
		      int x =  sample[i] + 0.2*(finger[i]-sample[i]);
		      sample[i] = (x<0)?0:(x>1000)?1000:x;
		      std::cout << i << ":" << sample[i] << " |  ";
		    }
		  std::cout << clk/1000.0 << "\xd" <<std::flush;
	      
		  // publish it
		  
		}
	    }
	  outlet.push_sample(sample);
	  usleep(1000);
	}
     
	
    }
  catch (std::exception& e)
    {
      std::cerr << "[ERROR] Got an exception: " << e.what() << std::endl;
    }
  

  close(glv_fd);
  return 0;
  
}
