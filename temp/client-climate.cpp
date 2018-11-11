#include <cstdio>
#include <stdio.h>
#include <cstdarg>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/sendfile.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>  
#include <fcntl.h>
#include <time.h>
#include <math.h>

#ifdef INTERACTIVE
#include <thread>
#endif

#define I2C_ADDR 0x04
#define BAUDRATE B9600
#define MODEMDEVICE "/dev/ttyAMA0"
#define BUF_SIZE 1024
#define MAXLINE 511
#define MAX_NAME_LENGTH 255
#define MAX_SIZE 100
#define COMMAND_SIZE 200
#include "scoped_exit.h"



static constexpr int CMD_BYTES = 19;
static constexpr int RESULT_BYTES = 10;
static constexpr int CMDID_MEAS = 0xC0;
static constexpr int CMDID_REPLY = 0xC5;

void error_handling(char *message);


namespace
{
	void error_message(const char* msg, ...)
	{
		va_list va;
		va_start(va, msg);
		char buff[512]{};
		sprintf(buff, "ERROR [%s]\n", msg);
		vfprintf(stderr, buff, va);
		va_end(va);
	}

	void dump(const uint8_t* pBuff, int num)
	{
		for (int i = 0; i < num; ++i) printf("%02x ", pBuff[i]);
	}

	std::string MakeTimestamp()
	{
		char buff[32]{};
		const time_t now{ time(nullptr) };
		strftime(buff, sizeof(buff), "%F.%T", gmtime(&now));
		return std::string(buff);
	}
}

namespace Cmd
{
	static constexpr int Head = 0;
	static constexpr int CmdID = 1;	// must be == CMDID_REPLY
	static constexpr int Type = 2;
	static constexpr int Data1 = Type;
	static constexpr int Data2 = 3;
	static constexpr int Data3 = 4;
	static constexpr int Data4 = 5;
	static constexpr int ID1 = 6;
	static constexpr int ID2 = 7;
	static constexpr int Checksum = 8;
	static constexpr int Tail = 9;
}

namespace Options
{
	// I need it simple and fast so found this on StackOverflow.com and altered a bit
	// http://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c
	class InputParser
	{
		std::vector <std::string> tokens;
	public:
		InputParser(int argc, const char** argv)
		{
			for (int i = 1; i < argc; ++i) tokens.emplace_back(argv[i]);
		}

		bool cmdOptionExists(const std::string& option) const
		{
			return std::find(tokens.begin(), tokens.end(), option) != tokens.end();
		}
	};

	static bool bInteractiveMode = false;
	static bool bJustNumbers = false;
}

std::string parseReply(const uint8_t(&V)[RESULT_BYTES])
{
	if (V[Cmd::CmdID] != CMDID_REPLY) return error_message("not a command reply"), "";

	char buff[512]{};

	switch (V[Cmd::Type])
	{
	case 6:	sprintf(buff, "mode %s: %s", V[Cmd::Data2] ? "set to" : "is", V[Cmd::Data3] ? "work" : "sleep"); break;
	case 8:	sprintf(buff, "work period %s: %d %s", V[Cmd::Data2] ? "set to" : "is", V[Cmd::Data3], V[Cmd::Data3] ? "minute(s)" : "(continous)"); break;
	default: sprintf(buff, "unknown command reply type [%02x]", V[Cmd::Type]);
	};

	return buff;
}

void parse(const uint8_t(&V)[RESULT_BYTES], bool bJustNumbers)
{
	if (V[Cmd::Head] != 0xAA) error_message("header mismatch");
	// 0xC0: result of measurement
	// 0xC5: command reply
	if (V[Cmd::CmdID] != CMDID_MEAS && V[Cmd::CmdID] != CMDID_REPLY) error_message("command prefix mismatch");
	if (V[Cmd::Tail] != 0xAB) error_message("tail mismatch");
	{
		int check = 0;
		for (int i = 2; i <= 7; ++i) check += V[i];
		if (V[Cmd::Checksum] != (check & 0xFF)) error_message("checksum mismatch");
	}

	if (V[Cmd::CmdID] == CMDID_MEAS)
	{
		const float PM2_5 = (V[3] * 256 + V[2]) / 10.f;
		const float PM10 = (V[5] * 256 + V[4]) / 10.f;
		if (bJustNumbers) printf("%4.01f %4.01f\n", PM2_5, PM10);
		else printf("-> MEAS PM25=%05.01f PM10=%05.01f ID=%02X%02X\n", PM2_5, PM10, V[Cmd::ID1], V[Cmd::ID2]);
	}
	else
	{
		if (!bJustNumbers) printf("-> RPLY d1=%02x, d2=%02x, d3=%02x, d4=%02x ID=%02X%02X <%s>\n", V[2], V[3], V[4], V[5], V[Cmd::ID1], V[Cmd::ID2], parseReply(V).c_str());
	}
}

namespace Request
{
	constexpr uint8_t GetPeriod[] = { 0xAA, 0xB4, 0x08, 0x00, 0x00, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x15, 0xAB };
	constexpr uint8_t SetPeriodCont[] = { 0xAA, 0xB4, 0x08, 0x01, 0x00, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x16, 0xAB };	// 0 == continuous mode
	constexpr uint8_t SetPeriod1m[] = { 0xAA, 0xB4, 0x08, 0x01, 0x01, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x17, 0xAB };	// [4] <- n (1-30) minutes
	constexpr uint8_t SetPeriod5m[] = { 0xAA, 0xB4, 0x08, 0x01, 0x05, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x1B, 0xAB };	// 5 minutes

	constexpr uint8_t GetMode[] = { 0xAA, 0xB4, 0x06, 0x00, 0x00, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x13, 0xAB };	// work
	constexpr uint8_t SetModeSleep[] = { 0xAA, 0xB4, 0x06, 0x01, 0x00, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x14, 0xAB };	// sleep
	constexpr uint8_t SetModeWork[] = { 0xAA, 0xB4, 0x06, 0x01, 0x01, 0,0,0,0,0,0,0,0,0,0, 0xAB, 0x62, 0x15, 0xAB };	// work
}


int set_interface_attribs(int fd, int speed, int parity)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0)
	{
		error_message("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN] = RESULT_BYTES;  // read doesn't block
	tty.c_cc[VTIME] = 0;            // 0 seconds read timeout (no timeout)

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		error_message("tcsetattr failed (errno=%d)", errno);
		return -1;
	}
	return 0;
}


bool SendCmd(const uint8_t(&V)[CMD_BYTES], int fd)
{
	int check = 0;
	for (int i = 2; i <= 16; ++i) check += V[i];
	check &= 0xFF;
	if (check != V[17]) error_message("mismatched checksum in command (%02x)", check);
	else
	{
		printf("Sending command: ");
		dump(V, sizeof(V));
		printf("\n");
		if (write(fd, V, sizeof(V)) != sizeof(V))
		{
			error_message("write failed (errno=%d)", errno);
			return false;
		}
	}
	return true;
}

char* ReadThread(int fd, int pip = -1)
{
	for(;;)
	{
		// this entire pip cruft is only used in interactive mode
		// to stop this thread in a correct, on-demand fashion
		if(pip != -1)
		{
			fd_set set;
			FD_ZERO(&set);
			FD_SET(fd, &set);
			FD_SET(pip, &set);

			if(select(FD_SETSIZE, &set, nullptr, nullptr, nullptr) == -1)
			{
				error_message("error from select\n");
				return NULL;
			}
			if(FD_ISSET(pip, &set))
			{
				// we got something through the pipe! we care not
				// what it is -> this is a signal we should return
				return NULL;
			}
			// if we're here it means there's data to be read from
			// the serial port - we just "fall-through"
		}

		uint8_t buf[RESULT_BYTES]{};
		const int nBytes = read(fd, buf, sizeof(buf));
		if(nBytes == RESULT_BYTES)
		{
			if(Options::bJustNumbers){ parse(buf, true);
				return buf;
			}
		}
		else
		{
			error_message("read failed with result=%d (errno=%d)", nBytes, errno);
			return NULL;
		}
	}
}

#ifdef INTERACTIVE
void Interactive(int fd)
{
	bool bOK = true;
	while(bOK)
	{
		switch(getchar())
		{
		case 'q':
		case 'Q': return;
		case 's': bOK = SendCmd(Request::SetModeSleep, fd); break;
		case 'w': bOK = SendCmd(Request::SetModeWork, fd); break;
		case 'p': bOK = SendCmd(Request::GetPeriod, fd); break;
		case 'c': bOK = SendCmd(Request::SetPeriodCont, fd); break;
		case '1': bOK = SendCmd(Request::SetPeriod1m, fd); break;
		case '5': bOK = SendCmd(Request::SetPeriod5m, fd); break;
		};
	}
}
#endif

//i2c functions



double HCHOcalculate(int raw,double R0){
	double Resistance = (1023.0/raw)-1;
	double calculated = pow(10.0,((log10(Resistance/R0)-0.0827)/(-0.4807)));

	return calculated;
}

double getCorfactor(float temp, float humidity){
	double factor = (0.00035*temp*temp - 0.02718*temp + 1.39538 - (humidity - 33.0)*0.0018);
	return factor;
}

double MQ135calculate(int raw, double R0){
	double Resistance = ((1023.0/raw)-1)*10.0;
	return	116.6020682 * pow((Resistance/getCorfactor(8,75)/R0), -2.769034857);
}

double MQ9calculate(int raw, double R0){

	   double sensor_volt=(double)raw/1024*5.0;
	   double RS_gas = (5.0-sensor_volt)/sensor_volt; // omit *RL

	   return RS_gas/R0;
}





//calibrate R0 value
#define HCHO_RO 34.28
#define MQ135_RO 76.63
#define MQ9_RO 7.87

char* i2cparser_cal(int fd){
		uint8_t V[8];
		char gas_value[BUF_SIZE];
		read(fd,V,8);
	 	int hcho = V[1]*256+V[2];
		int mq135 = V[3]*256+V[4];
		int mq9 = V[5]*256+V[6];
		sprintf(gas_value,"%.2lf %.2f %.2lf ",HCHOcalculate(hcho,HCHO_RO),MQ135calculate(mq135,MQ135_RO),MQ9calculate(mq9,MQ9_RO));

		return gas_value;


}




int main(int argc, char** argv)
{

	int sock;
	char message[BUF_SIZE];
	int str_len, recv_len, recv_cnt;
	struct sockaddr_in serv_adr;
	int hcho_R0;
	int mq132_RO;
	int mq9_RO;
	//machine information
	int mach_num = 1;
	// this program is for sending climate
	int type_num = 2;

	//for time
	time_t timer;
	struct tm *t;

	if (argc < 4) {
		printf("Usage : %s <IP> <port> SERIAL_PORT_PATH I2CPATH [OPTIONS]\n", argv[0]);
		printf("EXAMPLE:\t%s 192.168.100.216 3306 /dev/ttyAMA0\n\n", argv[0]);
		printf("OPTIONS:\n");
		printf("\t-i\tInteractive mode on (default is off)\n");
		printf("\t-n\tOutput only (space separated) values of PM2.5 first and PM10 later (default is full protocol dump)\n");
		printf("\n\n");
		exit(1);
	}


	//serial connection
	const char* portname = argv[3];

	Options::InputParser cmd(argc - 1, argv + 1);
	Options::bInteractiveMode = cmd.cmdOptionExists("-i");
	Options::bJustNumbers = cmd.cmdOptionExists("-n");

	const int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		const int err = errno;
		error_message("Cannot open '%s': %s (error=%d)", portname, strerror(err), err);
		return -1;
	}
	auto scopedUART = make_scoped([=] { close(fd); });
	set_interface_attribs(fd, B9600, 0);  //< set speed to 9600, 8n1 (no parity)

	if(!Options::bInteractiveMode) ReadThread(fd);

	//i2c
	int i2cfile;
	int i2caddr = 0x04;
	const char* i2cbus = argv[4];
	if((i2cfile = open(i2cbus,O_RDWR))<0) { //open i2cbus
		const int err = errno;
		error_message("Cannot open '%s': %s (error=%d)", i2cbus, strerror(err), err);
		return -1;
	}
	if(ioctl(i2cfile,I2C_SLAVE,i2caddr)<0){ //set i2c address
		const int err = errno;
				perror("i2caddress not responding");
				return -1;
	}


	//network
	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock == 1)
		error_handling("socket() error");

	memset(&serv_adr, 0, sizeof(serv_adr));
	serv_adr.sin_family = AF_INET;
	serv_adr.sin_addr.s_addr = inet_addr(argv[1]);
	serv_adr.sin_port = htons(atoi(argv[2]));

	if (connect(sock, (struct sockaddr*)&serv_adr, sizeof(serv_adr)) == -1)
		error_handling("connect() error!");
	else
		puts("Connected......");





	char time_now[BUF_SIZE];
	char year_now[BUF_SIZE];
	char month_now[BUF_SIZE];
	char day_now[BUF_SIZE];
	char hour_now[BUF_SIZE];
	char min_now[BUF_SIZE];
	char camera_shell[COMMAND_SIZE];
	char filename[MAX_NAME_LENGTH];
	char sds021_value[BUF_SIZE];
	char gas_value[BUF_SIZE];
	int k=0;
	while (1)
	{

		sprintf(filename,"image%4d.jpg", k);
		int width = 1920;
		int height = 1080;
		int delaytime = 10;

		//camera shell command to take pictures
		//example : raspistill -w 1920 -h 1080 -t 10 -o image.jpg
		sprintf(camera_shell, "raspistill -w %d -h %d -t %d -o %s", width, height, delaytime,filename);
		system(camera_shell);
		k++;


		if(!Options::bInteractiveMode){
			sds021_value = ReadThread(fd);
		}


		/*
		sending by input
		fputs("Input message(Q to quit) : ", stdout);
		fgets(message, BUF_SIZE, stdin);

		if(!strcmp(message, "q\n") || !strcmp(message, "Q\n"))
			break;
		*/

		// sending by automatic

		timer = time(NULL);
		t = localtime(&timer);

		////

		// int to string
		sprintf(year_now, "%d", t->tm_year + 1900);
		sprintf(month_now, "%d", t->tm_mon + 1);
		sprintf(day_now, "%d", t->tm_mday);
		sprintf(hour_now, "%d", t->tm_hour);
		sprintf(min_now, "%d", t->tm_min);

		strcpy(message, "0012"); // machine number + type number

		strcat(message, year_now);
		if (t->tm_mon + 1 < 10)
			strcat(message, "0");
		strcat(message, month_now);
		if (t->tm_mday < 10)
			strcat(message, "0");
		strcat(message, day_now);
		if (t->tm_hour < 10)
			strcat(message, "0");
		strcat(message, hour_now);
		if (t->tm_min < 10)
			strcat(message, "0");
		strcat(message, min_now);

		// sending sensor values
		strcat(message, sds021_value);
		strcat(message, i2cparser_cal(i2cfile));

		////


		str_len = write(sock, message, strlen(message));

		recv_len = 0;
		while (recv_len < str_len)
		{
			recv_cnt = read(sock, &message[recv_len], BUF_SIZE - 1);
			if (recv_cnt == -1)
				error_handling("read() error!");
			recv_len += recv_cnt;
		}

		message[recv_len] = 0;
		printf("Server received : %s", message);



		sleep(60);
	}
	close(sock);
	return 0;
}

void error_handling(char *message)
{
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}
