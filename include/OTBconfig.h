#ifndef OTBCONFIG_H
#define OTBCONFIG_H

#ifdef WIN32 
#include <winsock2.h> 
#elif defined (linux)
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h> // close 
#include <netdb.h> // gethostbyname
#define closesocket(s) close(s)
typedef int SOCKET;
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
typedef struct in_addr IN_ADDR;
#else
#error not defined for this platform
#endif

#define CONFIG_SIZE 40

#define ACQ_SETT 0b10000000
#define DECIM    0b01000000
#define REC_ON   0b00100000
#define FSAMP1   0b00010000
#define FSAMP0   0b00001000
#define NCH1     0b00000100
#define NCH0     0b00000010
#define ACQ_ON   0b00000001
#define ACQ_OFF  0b00000000

#define FSAMP_10240 FSAMP0 | FSAMP1
#define FSAMP_5120  FSAMP1
#define FSAMP_2048 FSAMP0
#define FSAMP_512 0b00000000

#define NCH_IN1to8_MIN1to4 NCH1 | NCH0
#define NCH_IN1to6_MIN1to3 NCH1
#define NCH_IN1to4_MIN1to2 NCH0
#define NCH_IN1to2_MIN1 0b00000000



#define AN_OUT_IN_SET 0b00000000
#define ANOUT_GAIN1   0b00100000
#define ANOUT_GAIN0   0b00010000
#define INSEL3        0b00001000
#define INSEL2        0b00000100
#define INSEL1        0b00000010
#define INSEL0        0b00000001

#define ANOUT_GAIN_16 ANOUT_GAIN1 | ANOUT_GAIN0
#define ANOUT_GAIN_4  ANOUT_GAIN1
#define ANOUT_GAIN_2  ANOUT_GAIN0
#define ANOUT_GAIN_1  0b00000000


#define AN_OUT_CH_SET 0b00000000
#define CHSEL5        0b00100000
#define CHSEL4        0b00010000
#define CHSEL3        0b00001000
#define CHSEL2        0b00000100
#define CHSEL1        0b00000010
#define CHSEL0        0b00000001


#define INX_CONF0 0b00000000
#define MUS6      0b01000000
#define MUS5      0b00100000
#define MUS4      0b00010000
#define MUS3      0b00001000
#define MUS2      0b00000100
#define MUS1      0b00000010
#define MUS0      0b00000001


#define Not_defined 0
#define Temporalis_Anterior 1
#define Superfic_Masseter 2
#define Splenius_Capitis 3
#define Upper_Trapezius 4
#define Middle_Trapezius 5
#define Lower_Trapezius 6
#define Rhomboideus_Major 7
#define Rhomboideus_Minor 8
#define Anterior_Deltoid 9
#define Posterior_Deltoid 10
#define Lateral_Deltoid 11
#define Infraspinatus 12
#define Teres_Major 13
#define Erector_Spinae 14
#define Latissimus_Dorsi 15
#define Bic_Br_Long_Head 16
#define Bic_Br_Short_Head 17
#define Tric_Br_Lat_Head 18
#define Tric_Br_Med_Head 19
#define Pronator_Teres 20
#define Flex_Carpi_Radial 21
#define Flex_Carpi_Ulnaris 22
#define Palmaris_Longus 23
#define Ext_Carpi_Radialis 24
#define Ext_Carpi_Ulnaris 25
#define Ext_Dig_Communis 26
#define Brachioradialis 27
#define Abd_Pollicis_Brev 28
#define Abd_Pollicis_Long 29
#define Opponens_Pollicis 30
#define Adductor_Pollicis 31
#define Flex_Poll_Brevis 32
#define Abd_Digiti_Minimi 33
#define Flex_Digiti_Minimi 34
#define Opp_Digiti_Minimi 35
#define Dorsal_Interossei 36
#define Palmar_Interossei 37
#define Lumbrical 38
#define Rectus_Abdominis 39
#define Ext_Abdom_Obliq 40
#define Serratus_Anterior 41
#define Pectoralis_Major 42
#define Sternoc_Ster_Head 43
#define Sternoc_Clav_Head 44
#define Anterior_Scalenus 45
#define Tensor_Fascia Latae 46
#define Gastrocn_Lateralis 47
#define Gastrocn_Medialis 48
#define Biceps_Femoris 49
#define Soleus 50
#define Semitendinosus 51
#define Gluteus_maximus 52
#define Gluteus_medius 53
#define Vastus_lateralis 54
#define Vastus_medialis 55
#define Rectus_femoris 56
#define Tibialis_anterior 57
#define Peroneus_longus 58
#define Semimembranosus 59
#define Gracilis 60
#define Ext_Anal_Sphincter 61
#define Puborectalis 62
#define Urethral_Sphincter 63
#define Not_a_Muscle 64

#define INX_CONF1 0b00000000
#define SENS4     0b10000000
#define SENS3     0b01000000
#define SENS2     0b00100000
#define SENS1     0b00010000
#define SENS0     0b00001000
#define ADAPT2    0b00000100
#define ADAPT1    0b00000010
#define ADAPT0    0b00000001

#define INX_CONF2 0b00000000
#define SIDE1     0b10000000
#define SIDE0     0b01000000
#define HPF1      0b00100000
#define HPF0      0b00010000
#define LPF1      0b00001000
#define LPF0      0b00000100
#define MODE1     0b00000010
#define MODE0     0b00000001

#define SIDE_NONE SIDE1 | SIDE0
#define SIDE_RIGHT SIDE1
#define SIDE_LEFT SIDE0
#define SIDE_UNDEFINED 0b00000000

#define HIGH_PASS_FILTER_200 HPF1 | HPF0
#define HIGH_PASS_FILTER_100 HPF1
#define HIGH_PASS_FILTER_10 HPF0
#define HIGH_PASS_FILTER_03 0b00000000

#define LOW_PASS_FILTER_4400 LPF1 | LPF0
#define LOW_PASS_FILTER_900 LPF1
#define LOW_PASS_FILTER_500 LPF0
#define LOW_PASS_FILTER_130 0b00000000

#define DETECTION_MODE_BIPOLAR MODE1
#define DETECTION_MODE_DIFFERENCIAL MODE0
#define DETECTION_MODE_MONOPOLAR 0b00000000

#define CRC_CODE 0b10001100



static void init(void);
static void end(void);
unsigned char crc(unsigned char config[]);
void printBIN(char);

#endif
