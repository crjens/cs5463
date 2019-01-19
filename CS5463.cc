#include <node.h>
#include <node_buffer.h>
#include <v8.h>

using namespace v8;
using namespace node;

#include <unistd.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <time.h>
#include "wiringPi.h"
#include "nan.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static uint8_t mode = 0;
static uint8_t bits = 8;
static int speed = 1000000;
static uint16_t delayTime = 0;
static int fd = 0;
static int SAMPLE_SIZE = 10;
static char * isrResultBuffer;
volatile static int isrSampleCount;
static int isrMaxSampleCount;
static struct timespec isrStartTime;
static int ISR_PIN = -1;
static sem_t semDataReady;
static char mask;

static char* Timestamp()
{
	static char ts[30];
	time_t ltime = time(NULL);
	struct tm * tm = localtime(&ltime);
	static struct timeval _t;
	static struct timezone tz;
	gettimeofday(&_t, &tz);

	sprintf(ts, "%04d-%02d-%02d %02d:%02d:%02d.%06d", tm->tm_year + 1900, tm->tm_mon, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, (int)_t.tv_usec);
	return ts;
}

static void errormsg(const char *s)
{
	fprintf(stderr, "(%s) Error: %s\n", Timestamp(), s);
	Nan::ThrowError(s);
}

static void warnmsg(const char *s)
{
	fprintf(stderr, "(%s) Warn: %s\n", Timestamp(), s);
}

struct timespec timer_start() {
	struct timespec start_time;
	clock_gettime(CLOCK_REALTIME, &start_time);
	return start_time;
}

int SendSpi(char * txBuffer, char * rxBuffer, int bufferLen)
{
	// Create Transfer Struct
	struct spi_ioc_transfer tr;
	memset(&tr, 0, sizeof(tr));
	tr.tx_buf = (unsigned long)txBuffer;
	tr.rx_buf = (unsigned long)rxBuffer;
	tr.len = bufferLen;
	tr.delay_usecs = delayTime;
	tr.speed_hz = speed;
	tr.bits_per_word = bits;

	// Send SPI-Message
	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret == -1) {
		printf("SendSpi error: %s\n", strerror(errno));
	}

	return ret;
}

long elapsedTime(struct timespec start_time) {
	struct timespec end_time;
	clock_gettime(CLOCK_REALTIME, &end_time);
	long diffInNanos = ((end_time.tv_sec - start_time.tv_sec) * 1E9) + (end_time.tv_nsec - start_time.tv_nsec);
	return diffInNanos;
}

void DisableInterrupts()
{
	mask = 0x0;
	// disable interrupts
	char tx[] = { 0x74, 0x00, 0x00, 0x00 };
	int ret = SendSpi(tx, tx, 4);
	if (ret < 1)
		errormsg("DisableInterrupts failed");

	//printf("interrupts disabled\n");
}

void EnableInterrupts(bool DRDY)
{
	mask = DRDY ? 0x80 : 0x10;

	char tx[] = { 0x5E, 0xFF, 0xFF, 0xFF,   // clear status
		0x74, mask, 0x00, 0x00 }; // enable/disable interrupts (0x74 = write to Mask) 
	int ret = SendSpi(tx, tx, 8);
	if (ret < 1)
		errormsg("EnableInterrupts failed");

	//printf("interrupts enabled:  %x\n", mask);
}

void IsrHandler(void)
{
	//printf("interrupt elasped: %ld\n", (long) (elapsedTime(isrStartTime) / 1E6));
	// handle interrupt here (connect DO pin on chip to ISR_PIN gpio input pin on pi)
	if (mask == 0x80)
	{
		char rx[] = { 0x1E, 0xFF, 0xFF, 0xFF, 0x5E, 0xFF, 0xFF, 0xFF };  // read status, clear status

		if (SendSpi(rx, rx, 8) < 1)
			errormsg("can't send spi message");

		if (rx[1] & 0x80) {
			if (sem_post(&semDataReady) == -1)
				errormsg("sem_post() failed");

			mask = 0x0;
		}
	}
	else if (mask == 0x10)
	{
		char tx[] = { 0x0E, 0xFF, 0xFF, 0xFF,	// read inst current
			0x10, 								// read inst voltage
			0x5E, 0xFF, 0xFF, 0xFF };			// clear status

		int ret = SendSpi(tx, tx, 9);

		if (ret < 1) {
			printf("can't read from Isr: %s\n", strerror(errno));
			//errormsg("can't read from Isr");
		}
		else
		{
			// cache results
			long elapsed = 0;

			if (isrSampleCount == 0)
			{
				isrStartTime = timer_start();
			}
			else
			{
				elapsed = elapsedTime(isrStartTime);
			}

			if (isrSampleCount >= isrMaxSampleCount/* || elapsed > 5E8*/)  // stop after buffer is full or .5 sec has elapsed 
			{
				//printf("finished collecting %d samples at: %ld\n", isrSampleCount, (long) (elapsed / 1E6));
				EnableInterrupts(true);
			}
			else
			{
				memcpy(isrResultBuffer + (isrSampleCount*SAMPLE_SIZE), tx + 1, 3);      // inst current
				memcpy(isrResultBuffer + (isrSampleCount*SAMPLE_SIZE) + 3, tx + 5, 3);  // inst voltage
				memcpy(isrResultBuffer + (isrSampleCount*SAMPLE_SIZE) + 6, &elapsed, 4);   // timestamp in ns
				isrSampleCount++;
			}
		}
	}
	else
	{
		char tx[] = { 0x5E, 0xFF, 0xFF, 0xFF };	// clear status
		SendSpi(tx, tx, 4);
	}
}

int OpenDevice(const char * device) {
	// Open Device and Check
	int ret = 0;

	fd = open(device, O_RDWR);
	if (fd < 0)
		errormsg("can't open device");

	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		errormsg("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		errormsg("can't get spi mode");

	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		errormsg("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		errormsg("can't get bits per word");

	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		errormsg("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		errormsg("can't get max speed hz");

	printf("Device opened with mode: %d, bits: %d, speed: %d\n", mode, bits, speed);

	return ret;
}

void CheckStatusResult(char * status, bool detailed)
{
	if (!(status[2] & 0x1))  // IC - invalid command (normally 1)
		errormsg("IC - Invalid command or status register not successfully read.");

	if (detailed) {
		if (status[2] & 0x4)  // LSD - Low supply detect
			warnmsg("LSD - Low supply detect");
		if (status[2] & 0x8)  // IOD - Modulator oscillation on current channel
			warnmsg("IOD - Modulator oscillation on current channel");
		if (status[2] & 0x10)  // VOD - Modulator oscillation on voltage channel
			warnmsg("VOD - Modulator oscillation on voltage channel");
		if (status[2] & 0x40)  // TOD - Modulator oscillation on temperature channel
			warnmsg("TOD - Modulator oscillation on temperature channel");

		if (status[1] & 0x4)  // VSAG - Voltage sag
			warnmsg("VSAG - Voltage sag");
		if (status[1] & 0x8)  // IFAULT - Current fault
			warnmsg("IFAULT - Current fault");
		if (status[1] & 0x10)  // EOR - Energy out of range
			warnmsg("EOR - Energy out of range");
		if (status[1] & 0x20)  // VROR - Vrms out of range
			warnmsg("VROR - Vrms out of range");
		if (status[1] & 0x40)  // IROR - Irms out of range
			warnmsg("IROR - Irms out of range");

		if (status[0] & 0x1)  // VOR - Voltage out of range
			warnmsg("VOR - Voltage out of range");
		if (status[0] & 0x2)  // IOR - Current out of range
			warnmsg("IOR - Current out of range");
	}
}

const char* ToCString(const v8::String::Utf8Value& value) {
	return *value ? *value : "<string conversion failed>";
}

void Close(const Nan::FunctionCallbackInfo<v8::Value>& info) {

	int ret = 0;
	//Close Device
	if (fd != 0) {

		ret = close(fd);
		printf("Closed device");
	}
	fd = 0;

	info.GetReturnValue().Set(Nan::New<Number>(ret));
}

void Open(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	int ret = 0;

	if (info.Length() < 1 || info.Length() > 5)
		errormsg("Expected 1-5 arguments - deviceName, (optional) speed, (optional) mode, (optional) bits, (optional) us delay");

	if (fd != 0)
		close(fd);

	Isolate* isolate = info.GetIsolate();
	String::Utf8Value str(isolate, info[0]);
	char* device = (char*)ToCString(str);

	if (info.Length() > 1)
		speed = Nan::To<int32_t>(info[1]).FromJust();

	if (info.Length() > 2)
		mode = Nan::To<int32_t>(info[2]).FromJust();

	if (info.Length() > 3)
		bits = Nan::To<int32_t>(info[3]).FromJust();

	if (info.Length() > 4)
		delayTime = (uint16_t)Nan::To<int32_t>(info[4]).FromJust();;

	printf("Opening device: %s at %d Hz, mode %d, %d bits, %d us delay\n", device, speed, mode, bits, (int)delayTime);
	ret = OpenDevice(device);

	info.GetReturnValue().Set(Nan::New<Number>(ret));
}

//Handle<Value> ReadCycle(const Arguments& args) {
//	HandleScope scope;
//
//	// make sure device is open    
//	if (fd == 0) {
//		errormsg(String::New("Must call Open first");
//	}
//
//	if (args.Length() != 1)
//		errormsg("Expected 1 argument - buffer");
//
//	int ret = 0;
//	struct timespec startTime;
//	long elapsed = 0;
//
//	char rx1[12], rx2[12];  // must be as big as the largest command
//	memset(rx1, 0, 12);
//	memset(rx2, 0, 12);
//
//	char txRead[12];
//	memset(txRead, 0xFF, 12);
//	txRead[0] = 0x1E;  // read status
//	txRead[4] = 0x0E;  // read inst current
//	txRead[8] = 0x10;  // read int voltage
//
//	char txClear[] = { 0x5E, 0xFF, 0xFF, 0xFF };  // clear status
//
//	char txStart[] = { 0xE8 };  // start continuous conversions
//	//char txStart[] = { 0xE0 };  // start single conversion
//
//	char txHalt[] = { 0xA0 };  // stop computations
//
//
//	char* out_buffer = NULL;
//	size_t out_length = -1;
//	Local<Object> out_buffer_obj;
//	out_buffer_obj = args[0]->ToObject();
//	out_buffer = Buffer::Data(out_buffer_obj);
//	out_length = Buffer::Length(out_buffer_obj);
//	int MAX_RESULTS = out_length / SAMPLE_SIZE;
//
//	//printf("SAMPLE_SIZE is %d bytes\n", SAMPLE_SIZE);
//	//printf("input buffer is %d bytes\n", out_length);
//	//printf("will collect a maximum of %d samples\n", MAX_RESULTS);
//
//	// clear status
//	ret = SendSpi(txClear, rx1, 4);
//	if (ret < 1)
//		errormsg("clear failed");
//
//	// start conversions
//	ret = SendSpi(txStart, rx1, 1);
//	if (ret < 1)
//		errormsg("start failed");
//
//	// swallow one cycle to let the filters settle
//	startTime = timer_start();
//	do
//	{
//		// read status
//		ret = SendSpi(txRead, rx1, 4);
//		if (ret < 1)
//			errormsg("can't send spi message");
//
//		CheckStatusResult(rx1 + 1, false);
//
//		elapsed = elapsedTime(startTime);
//
//	} while (!(rx1[1] & 0x80) && elapsed < 2E9);
//
//	// clear status
//	ret = SendSpi(txClear, rx1, 4);
//	if (ret < 1)
//		errormsg("clear failed");
//
//	int num = 0;
//	if (elapsed < 2E9) // only if we didn't time out above
//	{
//		char * rx;
//		long start;
//		startTime = timer_start();
//		do
//		{
//			// alternate receive buffers so we can compare with last value to see if anything changed
//			rx = num % 2 == 0 ? rx1 : rx2;
//
//			// read status, vInst and iInst
//			ret = SendSpi(txRead, rx, 12);
//			if (ret < 1)
//				errormsg("can't send spi message");
//
//			CheckStatusResult(rx + 1, true);
//
//			elapsed = elapsedTime(startTime);
//
//			//printf("num: %d, elapsed: %ld\n", num, elapsed)
//
//			// wait 10 ms for filters to settle before collecting samples
//			if (num < MAX_RESULTS && elapsed > 1E7 && (rx[1] & 0x10) && (0 != memcmp(rx1 + 5, rx2 + 5, 3) && 0 != memcmp(rx1 + 9, rx2 + 9, 3)))
//			{
//				if (num == 0)
//					start = elapsed;
//
//				long ts = elapsed - start;
//
//				memcpy(out_buffer + (num*SAMPLE_SIZE), rx + 5, 3);      // inst current
//				memcpy(out_buffer + (num*SAMPLE_SIZE) + 3, rx + 9, 3);  // inst voltage
//				memcpy(out_buffer + (num*SAMPLE_SIZE) + 6, &ts, 4);   // timestamp in ns
//				num++;
//			}
//		} while (!(rx[1] & 0x80) && elapsed < 2E9);
//	}
//
//	if (txStart[0] == 0xE8)
//	{
//		// halt conversions
//		ret = SendSpi(txHalt, rx1, 4);
//		if (ret < 1)
//			errormsg("halt failed");
//	}
//
//	if (elapsed >= 2E9)
//		return v8::Integer::New(-2);
//
//	return v8::Integer::New(num);
//}

int ReadCycleCount()
{
	char rx[] = { 0x0A, 0xFF, 0xFF, 0xFF };  // read cycle count

	// read cycle count register
	int ret = SendSpi(rx, rx, 4);
	if (ret < 1)
		errormsg("read cycle count failed");

	return (*(rx + 1) << 16) + (*(rx + 2) << 8) + *(rx + 3);
}

bool VerifyCycleCompleted()
{
	char rx[] = { 0x1E, 0xFF, 0xFF, 0xFF };  // read status

	if (SendSpi(rx, rx, 4) < 1)
		errormsg("can't send spi message");

	CheckStatusResult(rx + 1, true);
	//printf("rx[1]: %x\n", rx[1]);

	return (rx[1] & 0x80);
}

int Read(bool collectSamples)
{
	int ret = 0;
	struct timespec ts;

	// read cycle count register to calculate how long it should take for a cycle to complete
	int cycleCount = ReadCycleCount();
	long long cycleTime = 1000000000.0 * (cycleCount / 4000.0);

	// the first sample takes an additional 0.75 sec so add 1 sec to timeout
	long long timeout = 1000000000.0 + cycleTime;

	if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
		errormsg("clock_gettime");

	// normalize timespec for timeout	
	long long nsec = ts.tv_nsec + timeout;
	while (nsec >= 1000000000) {
		nsec -= 1000000000;
		++ts.tv_sec;
	}
	ts.tv_nsec = nsec;

	// Initialize semaphore
	if (sem_init(&semDataReady, 0, 0) == -1)
		errormsg("sem_init");

	EnableInterrupts(!collectSamples);

	while ((ret = sem_timedwait(&semDataReady, &ts)) == -1 && errno == EINTR) {
		continue;
	}

	if (ret == -1)
	{
		if (errno == ETIMEDOUT)
			printf("sem_timedwait() timed out\n");
		else
			perror("sem_timedwait error");
	}

	DisableInterrupts();

	if (sem_destroy(&semDataReady) == -1)
		errormsg("sem_destroy");

	return ret;
}


void ReadCycleWithInterrupts(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	// make sure device is open    
	if (fd == 0) {
		errormsg("Must call Open first");
	}

	if (info.Length() != 1)
		errormsg("Expected 1 argument - buffer");

	int ret = 0;
	char rx[4];  // must be as big as the largest command
	char txClear[] = { 0x5E, 0xFF, 0xFF, 0xFF };  // clear status
	char txStart[] = { 0xE8 };  // start continuous conversions
	char txHalt[] = { 0xA0 };  // stop computations
	
	Isolate* isolate = info.GetIsolate();
	Local<Context> context = isolate->GetCurrentContext();
  	Local<Object> out_buffer_obj = info[0]->ToObject(context).ToLocalChecked();

	char* out_buffer = NULL;
	size_t out_length = -1;
	out_buffer = Buffer::Data(out_buffer_obj);
	out_length = Buffer::Length(out_buffer_obj);
	int MAX_RESULTS = out_length / SAMPLE_SIZE;

	isrSampleCount = 0;
	isrMaxSampleCount = MAX_RESULTS;
	isrResultBuffer = out_buffer;

	//printf("SAMPLE_SIZE is %d bytes\n", SAMPLE_SIZE);
	//printf("input buffer is %d bytes\n", out_length);
	//printf("will collect a maximum of %d samples\n", MAX_RESULTS);

	// clear status
	ret = SendSpi(txClear, rx, 4);
	if (ret < 1)
		errormsg("clear failed");

	// start conversions
	ret = SendSpi(txStart, rx, 1);
	if (ret < 1)
		errormsg("start failed");

	// read one cycle and throw away to let filters settle
	ret = Read(false);
	if (ret != 0)
		errormsg("read initial cycle failed");

	// clear status
	ret = SendSpi(txClear, rx, 4);
	if (ret < 1)
		errormsg("clear failed");

	// read instantaneous values for full cycle
	ret = Read(true);
	if (ret != 0)
		errormsg("read full cycle failed");

	// halt conversions
	if (SendSpi(txHalt, rx, 1) < 1)
		errormsg("halt failed");

	if (ret != 0)
		return info.GetReturnValue().Set(Nan::New<Number>(ret));
	else
		return info.GetReturnValue().Set(Nan::New<Number>(isrSampleCount));
}

void Send(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	//Nan::EscapableHandleScope scope;

	// make sure device is open    
	if (fd == 0) {
		errormsg("Must call Open first");
	}
	int ret = 0;
	Isolate* isolate = info.GetIsolate();

	//Hexstring as first Argument. Convert it to Hex-Array
	String::Utf8Value str(isolate, info[0]);
	const char* hexinput = ToCString(str);

	int len = strlen(hexinput);

	uint8_t tx[len / 2];
	uint8_t rx[len / 2];
	size_t count = 0;

	for (count = 0; count < sizeof(tx) / sizeof(tx[0]); count++) {
		sscanf(hexinput, "%2hhx", &tx[count]);
		hexinput += 2 * sizeof(char);
	}

	// Send SPI-Message
	ret = SendSpi((char*)tx, (char*)rx, ARRAY_SIZE(tx));
	if (ret < 1)
		errormsg("can't send spi message");

	// Convert Return Values Back to a Hexstring
	unsigned int i;
	char* buf_str = (char*)malloc(2 * ARRAY_SIZE(rx) + 1);
	char* buf_ptr = buf_str;
	for (i = 0; i < ARRAY_SIZE(rx); i++)
	{
		buf_ptr += sprintf(buf_ptr, "%02X", rx[i]);
	}
	*(buf_ptr + 1) = '\0';
	//printf("send returned: %s\n", buf_str);

	info.GetReturnValue().Set(Nan::New(buf_str).ToLocalChecked());
	free(buf_str);
}

void Time(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	struct timespec startTime;
	int ret = 0;
	long elapsed = 0, min = 99999999, max = 0, last = 0, tm = 0;

	// make sure device is open    
	if (fd == 0) {
		errormsg("Must call Open first");
	}
	char buffer[5000];
	memset(buffer, 0xFF, 5000);

	char txRead[12], rx[12];  // must be as big as the largest command;
	memset(txRead, 0xFF, 12);
	txRead[0] = 0x1A;  // read epsilon (5A -> 01EB85)
	txRead[4] = 0x00;  // read config (000001)
	txRead[8] = 0x5E;  // clear status


	int iterations = 500;
	startTime = timer_start();
	for (int i = 0; i<iterations; i++)
	{
		ret = SendSpi(txRead, rx, 12);
		if (ret < 1)
			errormsg("can't send spi message");

		elapsed = elapsedTime(startTime) / 1000.0;
		tm = elapsed - last;
		last = elapsed;

		memcpy(buffer + (i * 10), rx + 1, 3);      // epsilon
		memcpy(buffer + (i * 10) + 3, rx + 5, 3);  // config
		memcpy(buffer + (i * 10) + 6, &elapsed, 4);   // timestamp in ns

		//printf("elapsed: %d,  last: %d, tm: %d\n", elapsed, last, tm);

		//printf("%02x %02x %02x %02x %02x %02x\n", *(rx+1), *(rx+2), *(rx+3), *(rx+5), *(rx+6), *(rx+7));

		if (tm > max)
			max = tm;
		if (tm < min)
			min = tm;
	}
	long telapsed = elapsedTime(startTime) / 1000.0;

	// check results
	bool success = true;
	for (int i = 0; i<iterations&&success; i++)
	{
		char * b = buffer + (i * 10);

		if (*(b) != 0x01 || *(b + 1) != 0xEB || *(b + 2) != 0x85 ||
			*(b + 3) != 0x00 || *(b + 4) != 0x00 || *(b + 5) != 0x01)
			success = false;
	}

	printf("%sTotal time for %d iterations was %ld us (%ld us per iteration), min: %ld, max: %ld\n", success ? "Success: " : "FAILED: ", iterations, telapsed, telapsed / iterations, min, max);

	info.GetReturnValue().Set(Nan::New<Number>(success ? 0 : -1));
}

void PinMode(const Nan::FunctionCallbackInfo<v8::Value>& info) {

	if (info.Length() != 2)
		errormsg("Expected 2 arguments - pinNum and direction (in=0, out=1)");

	int pin = Nan::To<int>(info[0]).FromJust();
	int mode = Nan::To<int>(info[1]).FromJust();

	pinMode(pin, mode);

	info.GetReturnValue().Set(Nan::New<Number>(0));
}

void DigitalWrite(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	if (info.Length() != 2)
		errormsg("Expected 2 arguments - pinNum and value (0 or 1)");

	int pin = Nan::To<int>(info[0]).FromJust();
	int value = Nan::To<int>(info[1]).FromJust();

	digitalWrite(pin, value);

	info.GetReturnValue().Set(Nan::New<Number>(0));
}

void DigitalPulse(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	if (info.Length() != 4)
		errormsg("Expected 4 arguments - pinNum, value1, value2 and delay time in microseconds");

	int pin = Nan::To<int>(info[0]).FromJust();
	int value1 = Nan::To<int>(info[1]).FromJust();
	int value2 = Nan::To<int>(info[2]).FromJust();
	int delay = Nan::To<int>(info[3]).FromJust();

	digitalWrite(pin, value1);
	//fprintf(stdout, "pin: %d set to: %d\n", pin, value1);

	delayMicroseconds(delay);
	digitalWrite(pin, value2);
	//fprintf(stdout, "pin: %d set to: %d\n", pin, value2);

	info.GetReturnValue().Set(Nan::New<Number>(0));
}

void InitializeISR(const Nan::FunctionCallbackInfo<v8::Value>& info)
{
	if (info.Length() < 1 || info.Length() > 3)
		errormsg("Expected 1-3 args, the interrupt pin number, pullup (0-off, 1-down, 2-up), isr edge(setup-0, falling-1, rising-2, both-3)");

	if (ISR_PIN == -1)
	{
		ISR_PIN = Nan::To<int>(info[0]).FromJust();
		int upDn = 0, edge = 1;
		if (info.Length() >1)
			upDn = Nan::To<int>(info[1]).FromJust();
		if (info.Length() >2)
			edge = Nan::To<int>(info[2]).FromJust();

		fprintf(stdout, "setting isr pin to: %d, upDn: %d, edge: %d\n", ISR_PIN, upDn, edge);

		pullUpDnControl(ISR_PIN, upDn);
		if (wiringPiISR(ISR_PIN, edge, &IsrHandler) < 0)
		{
			fprintf(stderr, "Unable to setup ISR: %s\n", strerror(errno));
			errormsg("Unable to setup ISR");
		}
	}

	info.GetReturnValue().Set(Nan::New<Number>(0));
}

NAN_MODULE_INIT(InitAll) {

	Nan::Set(target, Nan::New("Close").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(Close)).ToLocalChecked());
	Nan::Set(target, Nan::New("Open").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(Open)).ToLocalChecked());
	Nan::Set(target, Nan::New("ReadCycleWithInterrupts").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(ReadCycleWithInterrupts)).ToLocalChecked());
	Nan::Set(target, Nan::New("send").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(Send)).ToLocalChecked());
	Nan::Set(target, Nan::New("Time").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(Time)).ToLocalChecked());
	Nan::Set(target, Nan::New("PinMode").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(PinMode)).ToLocalChecked());

	Nan::Set(target, Nan::New("DigitalWrite").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(DigitalWrite)).ToLocalChecked());
	Nan::Set(target, Nan::New("DigitalPulse").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(DigitalPulse)).ToLocalChecked());
	Nan::Set(target, Nan::New("InitializeISR").ToLocalChecked(), Nan::GetFunction(Nan::New<v8::FunctionTemplate>(InitializeISR)).ToLocalChecked());

	if (wiringPiSetupPhys () < 0)
	{
		fprintf(stderr, "Unable to setup wiringPi: %s\n", strerror(errno));
		errormsg("Unable to setup wiringPi");
	}
}

NODE_MODULE(cs5463, InitAll)
