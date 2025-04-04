/* after start send data to USB serial port device, the watchdog is enabled, then you need remain send data each second
in case of USB disconnect/reconnect, will try reopen the port, if not success in time, the watchdog will press the ATX header reset button */

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int set_interface_attribs(int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
            printf("error %d from tcgetattr", errno);
            return -1;
        }
        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
            printf("error %d from tcsetattr", errno);
            return -1;
        }
        return 0;
}

int main(int argc, char *argv[])
{
    if(argc!=2)
    {
        std::cerr << "arg is serial port /dev/ttyUSB0" << std::endl;
        return -1;
    }
    int fd=-1;
    do
    {
        if(fd==-1)
        {
            fd=open(argv[1],O_RDWR|O_NOCTTY|O_SYNC);
            if(fd<0)
                std::cerr << "error " << errno << " opening " << argv[1] << std::endl;
            else
                set_interface_attribs(fd,B115200,0);
        }
        if(fd!=-1)
        {
            char var='1';
            const int r=write(fd,(char *)&var,1);
            if(r!=1)
            {
                std::cerr << "error writing, errno: " << errno << std::endl;
                ::close(fd);
                fd=-1;
            }
        }
        sleep(1);
    } while(true);

    return 0;
}
