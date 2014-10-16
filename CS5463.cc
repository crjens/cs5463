#include <node.h>
#include <node_buffer.h>
#include <v8.h>

using namespace v8;
using namespace node;

#include <unistd.h>
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
#include "wiringPi.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void errormsg(const char *s)
{
	ThrowException(Exception::Error(String::New(s)));
}

const char* ToCString(const v8::String::Utf8Value& value) {
  return *value ? *value : "<string conversion failed>";
}

long elapsedTime(struct timespec start_time) {
	struct timespec end_time;
	clock_gettime(CLOCK_REALTIME, &end_time);
	long diffInNanos = ((end_time.tv_sec - start_time.tv_sec) * 1E9) + (end_time.tv_nsec - start_time.tv_nsec);
	return diffInNanos;
}

struct timespec timer_start() {
	struct timespec start_time;
	clock_gettime(CLOCK_REALTIME, &start_time);
	return start_time;
}

static uint8_t mode = 0;
static uint8_t bits = 8;
static int speed = 1000000;
static uint16_t delayTime = 0;
static int fd=0;
static int SAMPLE_SIZE = 10;


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

    return ret;
}

Handle<Value>  Close(const Arguments& args) {
    int ret = 0;
    //Close Device
    if (fd != 0) {

        ret = close(fd);
		printf("Closed device");
	}
    fd=0;

    return v8::Integer::New(ret);
}

Handle<Value>  Open(const Arguments& args) 
{
    int ret = 0;

	if (args.Length() < 1 || args.Length() > 2)
		return ThrowException(Exception::TypeError(String::New("Expected 1 or 2 arguments - deviceName, (optional) speed")));

    if (fd != 0)
		close(fd);
	
	String::Utf8Value str(args[0]);
	char* device = (char*) ToCString(str);

	if (args.Length() == 2)
		speed = (int)(Local<Integer>::Cast(args[1])->Int32Value());

	printf("Opening device: %s at %d\n", device, speed);
   	ret = OpenDevice(device);
   
    return v8::Integer::New(ret);
}






int SendSpi(char * txBuffer, char * rxBuffer, int bufferLen)
{
	// Create Transfer Struct
    struct spi_ioc_transfer tr;
    tr.tx_buf = (unsigned long)txBuffer;
    tr.rx_buf = (unsigned long)rxBuffer;
    tr.len = bufferLen;
    tr.delay_usecs = delayTime;
    tr.speed_hz = speed;
    tr.bits_per_word = bits;

	// Send SPI-Message
	return ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
}


// Send Function
Handle<Value> ReadCycle(const Arguments& args) {
    HandleScope scope;

	// make sure device is open    
    if (fd == 0) {
		return ThrowException(Exception::TypeError(String::New("Must call Open first")));
    }

    if (args.Length() != 1)
		return ThrowException(Exception::TypeError(String::New("Expected 1 argument - buffer")));

    int ret = 0;
    struct timespec startTime;
    long elapsed =0;

    char rx1[12], rx2[12];  // must be as big as the largest command
	memset(rx1, 0, 12);
	memset(rx2, 0, 12);

    char txRead[12];
    memset(txRead, 0xFF, 12);
    txRead[0] = 0x1E;  // read status
    txRead[4] = 0x0E;  // read inst current
    txRead[8] = 0x10;  // read int voltage

	char txClear[] = { 0x5E, 0xFF, 0xFF, 0xFF};  // clear status
    
	char txStart[] = { 0xE8 };  // start continuous conversions
	//char txStart[] = { 0xE0 };  // start single conversion

	char txHalt[] = { 0xA0 };  // stop computations


    char* out_buffer = NULL;
    size_t out_length = -1;
    Local<Object> out_buffer_obj;
    out_buffer_obj = args[0]->ToObject();
    out_buffer = Buffer::Data(out_buffer_obj);
    out_length = Buffer::Length(out_buffer_obj);
    int MAX_RESULTS = out_length / SAMPLE_SIZE;

    //printf("SAMPLE_SIZE is %d bytes\n", SAMPLE_SIZE);
    //printf("input buffer is %d bytes\n", out_length);
    //printf("will collect a maximum of %d samples\n", MAX_RESULTS);

    // clear status
    ret = SendSpi(txClear, rx1, 4);
    if (ret < 1)
        errormsg("clear failed");

    // start conversions
    ret = SendSpi(txStart, rx1, 1);
    if (ret < 1)
        errormsg("start failed");

    // swallow one cycle to let the filters settle
	startTime = timer_start();
	do 
    {
		// read status
		ret = SendSpi(txRead, rx1, 4);
		if (ret < 1)
			errormsg("can't send spi message");

        elapsed = elapsedTime(startTime);

    } while (!(rx1[1] & 0x80) && elapsed < 2E9);

	// clear status
    ret = SendSpi(txClear, rx1, 4);
    if (ret < 1)
        errormsg("clear failed");

	int num=0;
	if (elapsed < 2E9)
	{
		
		char * rx;
		long start;
		startTime = timer_start();
		do 
		{
			// alternate receive buffers so we can compare with last value to see if anything changed
			rx = num%2==0 ? rx1 : rx2;  

			// read status, vInst and iInst
			ret = SendSpi(txRead, rx, 12);
			if (ret < 1)
				errormsg("can't send spi message");

			elapsed = elapsedTime(startTime);

			// wait 10 ms for filters to settle before collecting samples
			if (num < MAX_RESULTS && elapsed > 1E7 && (rx[1] & 0x10) && (0 != memcmp(rx1+5, rx2+5, 3) && 0 != memcmp(rx1+9, rx2+9, 3)))
			{
				if (num == 0)
					start = elapsed;

				long ts = elapsed - start;

				memcpy(out_buffer + (num*SAMPLE_SIZE), rx+5, 3);      // inst current
				memcpy(out_buffer + (num*SAMPLE_SIZE) + 3, rx+9, 3);  // inst voltage
				memcpy(out_buffer + (num*SAMPLE_SIZE) + 6, &ts, 4);   // timestamp in ns
				num++;
			} 
		} while (!(rx[1] & 0x80) && elapsed < 2E9);
	}

	if (txStart[0] == 0xE8)
	{
		// halt conversions
		ret = SendSpi(txHalt, rx1, 4);
		if (ret < 1)
			errormsg("halt failed");
	}

    if (elapsed >= 2E9)
		return v8::Integer::New(-2);

    return v8::Integer::New(num);
}

static char * isrResultBuffer;
volatile static int isrSampleCount;
static int isrMaxSampleCount;
static struct timespec isrStartTime;
static int ISR_PIN = -1;

void DisableInterrupts()
{
	// disable interrupts
	char tx[] = { 0x74, 0x00, 0x00, 0x00};
	SendSpi(tx, tx, 4);
}

void EnableInterrupts(char * buffer, int maxSamples)
{
	isrSampleCount = 0;
	isrMaxSampleCount = maxSamples;
	isrResultBuffer = buffer;
	
	char tx[] = {	0x5E, 0xFF, 0xFF, 0xFF,   // clear status
					0x74, 0x10, 0x00, 0x00 }; // enable interrupts
	SendSpi(tx, tx, 8);
}

Handle<Value> ReadCycleWithInterrupts(const Arguments& args) {
    HandleScope scope;

	// make sure device is open    
    if (fd == 0) {
		return ThrowException(Exception::TypeError(String::New("Must call Open first")));
    }

    if (args.Length() != 1)
		return ThrowException(Exception::TypeError(String::New("Expected 1 argument - buffer")));

    int ret = 0;
    struct timespec startTime;
    long elapsed =0;

    char rx[12];  // must be as big as the largest command

    char txRead[12];
    memset(txRead, 0xFF, 12);
    txRead[0] = 0x1E;  // read status
    txRead[4] = 0x0E;  // read inst current
    txRead[8] = 0x10;  // read int voltage

	char txClear[] = { 0x5E, 0xFF, 0xFF, 0xFF};  // clear status
    
	char txStart[] = { 0xE8 };  // start continuous conversions
	//char txStart[] = { 0xE0 };  // start single conversion

	char txHalt[] = { 0xA0 };  // stop computations


    char* out_buffer = NULL;
    size_t out_length = -1;
    Local<Object> out_buffer_obj;
    out_buffer_obj = args[0]->ToObject();
    out_buffer = Buffer::Data(out_buffer_obj);
    out_length = Buffer::Length(out_buffer_obj);
    int MAX_RESULTS = out_length / SAMPLE_SIZE;

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

	// swallow one cycle to let the filters settle
	startTime = timer_start();
	do 
    {
		// read status
		ret = SendSpi(txRead, rx, 4);
		if (ret < 1)
			errormsg("can't send spi message");

        elapsed = elapsedTime(startTime);

    } while (!(rx[1] & 0x80) && elapsed < 2E9);

	// clear status
    ret = SendSpi(txClear, rx, 4);
    if (ret < 1)
        errormsg("clear failed");

    // read instantaneuos values for full cycle
	bool doInterrupts = true;
	int num=0;
	long start;
    startTime = timer_start();
    do 
    {
		// read status, vInst and iInst
		ret = SendSpi(txRead, rx, 12);
		if (ret < 1)
			errormsg("can't send spi message");

        elapsed = elapsedTime(startTime);

		// Check CRDY bit
		if (elapsed > 1E7 && doInterrupts && (rx[1] & 0x10))
		{
			doInterrupts = false;

			// start interrupt handler
			//printf("enable ints\n");
			EnableInterrupts(out_buffer, MAX_RESULTS);

			// sleep for 500 ms
			usleep(500000);

			DisableInterrupts();
			//printf("disable ints: %d\n", isrSampleCount);
		}


		// wait 10 ms for filters to settle before collecting samples
        /*if (elasped > 1E7 && num < MAX_RESULTS && (rx[1] & 0x40) && (!memcmp(rx1+5, rx2+5, 3) || !memcmp(rx1+9, rx2+9, 3)))
        {
			if (num == 0)
				start = elapsed;
			long ts = elapsed - start;

			memcpy(out_buffer + (num*SAMPLE_SIZE), rx+5, 3);      // inst current
			memcpy(out_buffer + (num*SAMPLE_SIZE) + 3, rx+9, 3);  // inst voltage
			memcpy(out_buffer + (num*SAMPLE_SIZE) + 6, &ts, 4);   // timestamp in ns
			num++;
		} */
    } while (!(rx[1] & 0x80) && elapsed < 2E9);

	if (txStart[0] == 0xE8)
	{
		// halt conversions
		ret = SendSpi(txHalt, rx, 4);
		if (ret < 1)
			errormsg("halt failed");
	}

    if (elapsed >= 2E9)
		return v8::Integer::New(-2);

    return v8::Integer::New(isrSampleCount);
}

// Send Function
Handle<Value> Send(const Arguments& args) {
    HandleScope scope;
	
	// make sure device is open    
    if (fd == 0) {
		return ThrowException(Exception::TypeError(String::New("Must call Open first")));
    }
    int ret = 0;

    //Hexstring as first Argument. Convert it to Hex-Array
    String::Utf8Value str(args[0]);
    const char* hexinput = ToCString(str);

    int len = strlen(hexinput);

    uint8_t tx[len / 2];
    uint8_t rx[len / 2];
    size_t count = 0;

    for(count = 0; count < sizeof(tx)/sizeof(tx[0]); count++) {
        sscanf(hexinput, "%2hhx", &tx[count]);
        hexinput += 2 * sizeof(char);
    }

    /* debug */
    /*
    for(count = 0; count < sizeof(tx)/sizeof(tx[0]); count++)
      printf("%02x:", tx[count]);
    printf("\n");
    */

	// Create Transfer Struct
    struct spi_ioc_transfer tr;
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = ARRAY_SIZE(tx);
    tr.delay_usecs = delayTime;
    tr.speed_hz = speed;
    tr.bits_per_word = bits;


    // Send SPI-Message
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
        errormsg("can't send spi message");

    // Convert Return Values Back to a Hexstring
    unsigned int i;
    char* buf_str = (char*) malloc (2*ARRAY_SIZE(rx)+1);
    char* buf_ptr = buf_str;
    for (i = 0; i < ARRAY_SIZE(rx); i++)
    {
       buf_ptr += sprintf(buf_ptr, "%02X", rx[i]);
    }
    *(buf_ptr + 1) = '\0';
    return scope.Close(String::New(buf_str));
}


Handle<Value> Time(const Arguments& args) {
	struct timespec startTime;
	int ret=0;
	long elapsed=0, min=99999999, max = 0, last=0, tm=0;

	// make sure device is open    
    if (fd == 0) {
		return ThrowException(Exception::TypeError(String::New("Must call Open first")));
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
	for (int i=0;i<iterations; i++)
	{
		ret = SendSpi(txRead, rx, 12);
		if (ret < 1)
			errormsg("can't send spi message");
		
		elapsed = elapsedTime(startTime)/1000.0;
		tm = elapsed - last;
		last = elapsed;

		memcpy(buffer + (i*10), rx+1, 3);      // epsilon
		memcpy(buffer + (i*10) + 3, rx+5, 3);  // config
		memcpy(buffer + (i*10) + 6, &elapsed, 4);   // timestamp in ns

		//printf("elapsed: %d,  last: %d, tm: %d\n", elapsed, last, tm);

		//printf("%02x %02x %02x %02x %02x %02x\n", *(rx+1), *(rx+2), *(rx+3), *(rx+5), *(rx+6), *(rx+7));

		if (tm > max)
			max = tm;
		if (tm < min)
			min = tm;
	}
	long telapsed = elapsedTime(startTime)/1000.0;



	// check results
	bool success = true;
	for (int i=0;i<iterations&&success;i++)
	{
		char * b = buffer + (i*10);

		if (*(b) != 0x01 || *(b+1) != 0xEB || *(b+2) != 0x85 || 
			*(b+3) != 0x00 || *(b+4) != 0x00 || *(b+5) != 0x01)
			success = false;
	}

	printf("%sTotal time for %d iterations was %d us (%d us per iteration), min: %d, max: %d\n", success ? "Success: " : "FAILED: " , iterations, telapsed, telapsed / iterations, min, max);

	 return v8::Integer::New(success ? 0 : -1);
}

// GPIO stuff
Handle<Value> PinMode(const Arguments& args) {
    
	if (args.Length() != 2)
		return ThrowException(Exception::TypeError(String::New("Expected 2 arguments - pinNum and direction (in=0, out=1)")));

	int pin = (int)(Local<Integer>::Cast(args[0])->Int32Value());
	int mode = (int)(Local<Integer>::Cast(args[1])->Int32Value());

	pinMode(pin, mode);

    return v8::Integer::New(0);
}

Handle<Value> DigitalWrite(const Arguments& args) {
    if (args.Length() != 2)
		return ThrowException(Exception::TypeError(String::New("Expected 2 arguments - pinNum and value (0 or 1)")));

	int pin = (int)(Local<Integer>::Cast(args[0])->Int32Value());
	int value = (int)(Local<Integer>::Cast(args[1])->Int32Value());

	digitalWrite(pin, value);

    return v8::Integer::New(0);
}

Handle<Value> DigitalPulse(const Arguments& args) {
    if (args.Length() != 4)
		return ThrowException(Exception::TypeError(String::New("Expected 4 arguments - pinNum, value1, value2 and delay time in microseconds")));

	int pin = (int)(Local<Integer>::Cast(args[0])->Int32Value());
	int value1 = (int)(Local<Integer>::Cast(args[1])->Int32Value());
	int value2 = (int)(Local<Integer>::Cast(args[2])->Int32Value());
	int delay = (int)(Local<Integer>::Cast(args[3])->Int32Value());
		
	digitalWrite(pin, value1);
	printf("pin: %d set to: %d\n", pin, value1);

	delayMicroseconds(delay);
	digitalWrite(pin, value2);
	printf("pin: %d set to: %d\n", pin, value2);

    return v8::Integer::New(0);
}

void IsrHandler2(void)
{
	//printf("interrupt\n");
	// handle interrupt here (connect DO pin on chip to ISR_PIN gpio input pin on pi)
	char tx[] = {   0x0E, 0xFF, 0xFF, 0xFF,   // read inst current
					0x10, 0xFF, 0xFF, 0xFF,   // read int voltage
					0x5E, 0xFF, 0xFF, 0xFF};  // clear status
	int ret = SendSpi(tx, tx, 12);
	if (ret < 1) {
		printf ("can't read from Isr: %s\n", strerror (errno)) ;
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
		
		if (isrSampleCount >= isrMaxSampleCount || elapsed > 5E8)  // stop after buffer is full or .5 sec has elapsed 
		{
			// maybe enable interrupt for DRDY ????
			DisableInterrupts();	
		}
		else
		{
			memcpy(isrResultBuffer + (isrSampleCount*SAMPLE_SIZE), tx+1, 3);      // inst current
			memcpy(isrResultBuffer + (isrSampleCount*SAMPLE_SIZE) + 3, tx+5, 3);  // inst voltage
			memcpy(isrResultBuffer + (isrSampleCount*SAMPLE_SIZE) + 6, &elapsed, 4);   // timestamp in ns
			isrSampleCount++;
		}
	}
}

Handle<Value> InitializeISR(const Arguments& args) {
    if (args.Length() < 1 || args.Length() > 3)
		return ThrowException(Exception::TypeError(String::New("Expected 1-3 args, the interrupt pin number, pullup (0-off, 1-down, 2-up), isr edge(setup-0, falling-1, rising-2, both-3)")));

	if (ISR_PIN == -1)
	{
		ISR_PIN = (int)(Local<Integer>::Cast(args[0])->Int32Value());
		int upDn =0, edge =1;
		if (args.Length() >1)
			upDn = (int)(Local<Integer>::Cast(args[1])->Int32Value());
		if (args.Length() >2)
			edge = (int)(Local<Integer>::Cast(args[2])->Int32Value());

		printf("setting isr pin to: %d, upDn: %d, edge: %d\n", ISR_PIN, upDn, edge);

		pullUpDnControl (ISR_PIN, upDn);
		if (wiringPiISR(ISR_PIN, edge, &IsrHandler2) < 0)
		{
			fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno)) ;
			errormsg("Unable to setup ISR");
		}
	}

    return v8::Integer::New(0);
}


void init(Handle<Object> exports) {

    exports->Set(String::NewSymbol("Open"), FunctionTemplate::New(Open)->GetFunction());
    exports->Set(String::NewSymbol("Close"), FunctionTemplate::New(Close)->GetFunction());
    exports->Set(String::NewSymbol("send"), FunctionTemplate::New(Send)->GetFunction());
    exports->Set(String::NewSymbol("ReadCycle"), FunctionTemplate::New(ReadCycle)->GetFunction());
	exports->Set(String::NewSymbol("PinMode"), FunctionTemplate::New(PinMode)->GetFunction());
	exports->Set(String::NewSymbol("DigitalWrite"), FunctionTemplate::New(DigitalWrite)->GetFunction());
	exports->Set(String::NewSymbol("DigitalPulse"), FunctionTemplate::New(DigitalPulse)->GetFunction());
	exports->Set(String::NewSymbol("Time"), FunctionTemplate::New(Time)->GetFunction());
	exports->Set(String::NewSymbol("InitializeISR"), FunctionTemplate::New(InitializeISR)->GetFunction());
	exports->Set(String::NewSymbol("ReadCycleWithInterrupts"), FunctionTemplate::New(ReadCycleWithInterrupts)->GetFunction());
	

	if (wiringPiSetup() < 0)
	{
		fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno)) ;
		errormsg("Unable to setup wiringPi");
	}

	/*if (piHiPri(99) < 0)
	{
		fprintf (stderr, "Unable to set thread priority: %s\n", strerror (errno)) ;
		errormsg("Unable to set thread priority");
	}*/
}


NODE_MODULE(cs5463, init)