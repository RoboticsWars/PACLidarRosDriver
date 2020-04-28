#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include "ioapi.h"
#include "bufferDecode.h"

namespace lidar_driver
{

static const int POLL_TIMEOUT = 200; // one second (in msec)

////////////////////////////////////////////////////////////////////////
// base class implementation
////////////////////////////////////////////////////////////////////////
IOAPI::IOAPI()
{
    m_bufferPro = new  SSBufferDec();
}

IOAPI::~IOAPI() {
    if(m_bufferPro) {
        m_bufferPro->exit();
        delete m_bufferPro;
        m_bufferPro = NULL;
    }
}


int IOAPI::revPacket()
{
    int rtn = 0 ;
    int writeidx = 0 ;
    if( !m_bufferPro->isFull() ) {

        writeidx = 0 ;

        while( writeidx < LIDAR_DATA_PACKAGE_SIZE ) {
            rtn = ioRead(m_bufferPro->getWriteIndex()+writeidx, LIDAR_DATA_PACKAGE_SIZE-writeidx ) ;
            if(rtn>0) {
                writeidx += rtn;
                if(writeidx == LIDAR_DATA_PACKAGE_SIZE) {
                    m_bufferPro->nextWriteIndex();

                    break;
                } else if( writeidx > LIDAR_DATA_PACKAGE_SIZE ) {
                    writeidx = 0 ;
                    ROS_INFO("writeidx > LIDAR_DATA_PACKAGE_SIZE\n");
                }
            } else {
                return rtn;
            }
        }

    }

    //    if(m_bufferPro->isFull()) {
    //        ROS_INFO("m_bufferPro->isFull\n");
    //        m_bufferPro->printIndex();
    //    }
    return rtn ;
}

/** @brief Get packet. */
int IOAPI::getPacket() {
    int rtn = 0;
    rtn = m_bufferPro->readPacket() ;
    return rtn;
}

////////////////////////////////////////////////////////////////////////
// UDPSocketAPI class implementation
////////////////////////////////////////////////////////////////////////

///
/// \brief UDPSocketAPI::UDPSocketAPI
/// \param ipstr
/// \param devport
/// \param pcport
///
UDPSocketAPI::UDPSocketAPI(std::string ipstr, uint16_t devport, int16_t pcport)
{
    m_sockfd = -1;
    //  ROS_INFO_STREAM("Opening UDP socket: " <<
    //                  " pc port "            << pcport<<
    //                  " device port  "       << devport<<
    //                  " device ip  "         << ipstr );
    m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    if (m_sockfd == -1) {
        //perror(" create socket error");
        return;
    }

    sockaddr_in my_addr;
    memset(&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(pcport);
    my_addr.sin_addr.s_addr = INADDR_ANY;

    memset(&m_devaddr, 0, sizeof(m_devaddr));
    m_devaddr.sin_family = AF_INET;
    m_devaddr.sin_port = htons(devport);
    m_devaddr.sin_addr.s_addr = inet_addr(ipstr.c_str() );

    if (bind(m_sockfd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
        //perror("bind message");
        return;
    }

    if (fcntl(m_sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        //perror("non-block message");
        return;
    }

    reset();
}

///
/// \brief UDPSocketAPI::~UDPSocketAPI
///
UDPSocketAPI::~UDPSocketAPI(void)
{
    (void)close(m_sockfd);
}

///
/// \brief UDPSocketAPI::write
/// \param data
/// \param size
/// \return
///
int UDPSocketAPI::ioWrite(unsigned char *data, int size)
{
    int rtn = 0 ;
    unsigned char tmpCmd[UDP_FRAME_MIN] ;
    unsigned char *tmpBuf = data ;
    socklen_t sender_address_len = sizeof(m_devaddr);
    if(m_sockfd <= 0) return rtn ;

    if (size < UDP_FRAME_MIN && size > 0) {
        memcpy(tmpCmd, data, size);
        tmpBuf = tmpCmd;
        size = UDP_FRAME_MIN;
    }

    rtn = sendto(m_sockfd, data, size, 0, (sockaddr*)&m_devaddr, sender_address_len ) ;
    if (rtn < 0) {
        //perror("IOSocketAPI:write") ;
    }
    return rtn ;
}

///
/// \brief UDPSocketAPI::reset
/// \return
///
int UDPSocketAPI::reset()
{
    m_bufferPro->reset();
    return 0;
}

///
/// \brief UDPSocketAPI::read
/// \param data
/// \param size
/// \return
///
int UDPSocketAPI::ioRead(unsigned char *data, int size)
{
    struct pollfd fds[1];
    fds[0].fd = m_sockfd;
    fds[0].events = POLLIN;
    int nbytes = 0 ;
    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    int retval = 0 ;
    retval = poll(fds, 1, POLL_TIMEOUT);
    if(fds[0].revents & POLLIN) {
        nbytes = recvfrom(m_sockfd, data, size, 0,
                          (sockaddr*)&sender_address, &sender_address_len);
        if(m_devaddr.sin_addr.s_addr != sender_address.sin_addr.s_addr) {
            nbytes = 0 ;
        }
    }
    if (retval == 0) {
        ROS_WARN("IOSocketAPI::read  poll() timeout");
        nbytes = 0;
    }
    return nbytes ;
}

////////////////////////////////////////////////////////////////////////
// TCPSocketAPI class implementation
////////////////////////////////////////////////////////////////////////
TCPSocketAPI::TCPSocketAPI(string ipstr, uint16_t devport, int16_t pcport)
{
    m_sockfd =  0;
    m_devip_str = ipstr;
    m_devport = devport;
    m_pcport = pcport;
    reset();

}

TCPSocketAPI::~TCPSocketAPI()
{
    closeSocket();
}

///
/// \brief TCPSocketAPI::write
/// \param data
/// \param size
/// \return
///
int TCPSocketAPI::ioWrite(unsigned char *data, int size)
{
    size_t		nleft;
    ssize_t		nwritten;

    nleft = size;
    while (nleft > 0) {
        if ( (nwritten = write(m_sockfd, data, nleft)) <= 0) {
            if (nwritten < 0 && errno == EINTR)
                nwritten = 0;		/* and call write() again */
            else {
                close(m_sockfd);
                m_sockfd = 0 ;
                ROS_INFO("ioWrite error\n");
                return(nwritten);			/* error */
            }

        }
        nleft -= nwritten;
        data   += nwritten;
    }
    usleep(50000);
    return (size);
}

///
/// \brief TCPSocketAPI::read
/// \param data
/// \param size
/// \return
///
int TCPSocketAPI::ioRead(unsigned char *data, int size)
{
    int	n = -1 ;
    if(m_sockfd <=0 ) {
        ROS_INFO("read error\n");
        return n;
    }
    //    if ( (n = read(m_sockfd, data, size)) <= 0) {
    //        if (n == 0)
    //            return n;
    //        else {
    //            ROS_INFO("ioRead error\n");
    //        }
    //    }

    struct pollfd fds[1];
    fds[0].fd = m_sockfd;
    fds[0].events = POLLIN;
    int retval = 0 ;
    retval = poll(fds, 1, POLL_TIMEOUT);
    if(fds[0].revents & POLLIN) {
        if ( (n = read(m_sockfd, data, size)) <= 0) {
            if (n == 0)
                return n;
            else {
                ROS_INFO("ioRead error\n");
            }
        }
    } else if(fds[0].revents & POLLERR) {
        ROS_WARN("TCPSocketAPI::ioRead  poll() POLLERR ");
    }

    if (retval == 0) {
        ROS_WARN("TCPSocketAPI::ioRead  poll() timeout");
    } else if(retval == -1) {
        ROS_WARN("TCPSocketAPI::ioRead  poll() error");
    }
    return n;
}

///
/// \brief TCPSocketAPI::reset
/// \return
///
int TCPSocketAPI::reset()
{
    if( m_sockfd ) {
        close(m_sockfd);
        m_sockfd = 0 ;
    }

    struct sockaddr_in mine;


    if ( (m_sockfd = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        //close(m_sockfd);
        m_sockfd = 0 ;
        ROS_INFO("socket error\n");
        return m_sockfd;
    }

    bzero(&mine,sizeof(mine));
    mine.sin_family = AF_INET;
    mine.sin_port = htons(m_pcport);
    mine.sin_addr.s_addr = INADDR_ANY;
    // int b = bind(m_sockfd,(struct sockaddr*)&mine,sizeof(mine));
    //if(b ==-1 ){
    // close(m_sockfd);
    // m_sockfd = 0 ;
    // ROS_INFO("bind m_sockfd:");
    //return m_sockfd;
    //}

    //    if(listen(m_sockfd,5) == -1) {
    //        ROS_INFO("error listening on socket\n");

    //    }


    bzero(&m_servaddr, sizeof(m_servaddr));
    m_servaddr.sin_family      = AF_INET;
    m_servaddr.sin_addr.s_addr = inet_addr(m_devip_str.c_str() );
    m_servaddr.sin_port        = htons(m_devport);            /* daytime server */

    if (connect_nonb(m_sockfd, (SA *) &m_servaddr, sizeof(m_servaddr), 2) < 0) {
        close(m_sockfd);
        m_sockfd = 0;
        ROS_INFO("connect error\n");
    }
    return m_sockfd;
}

///
/// \brief TCPSocketAPI::TCPFcntl
/// \param fd
/// \param cmd
/// \param arg
/// \return
///
int TCPSocketAPI::TCPFcntl(int fd, int cmd, int arg)
{
    int	n;

    if ( (n = fcntl(fd, cmd, arg)) == -1)
        ROS_INFO("fcntl error\n");
    return (n);
}

//int
//Select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
//       struct timeval *timeout)
//{
//    int		n;

//    if ( (n = select(nfds, readfds, writefds, exceptfds, timeout)) < 0)
//        printf("select error\n");

//    return(n);		/* can return 0 on timeout */
//}

///
/// \brief TCPSocketAPI::connect_nonb
/// \param sockfd
/// \param saptr
/// \param salen
/// \param nsec
/// \return
///
int TCPSocketAPI::connect_nonb(int sockfd, const SA *saptr, socklen_t salen, int nsec)
{
    int				flags, n, error;
    socklen_t		len;
    fd_set			rset, wset;
    struct timeval	tval;

    flags = TCPFcntl(sockfd, F_GETFL, 0);

    TCPFcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    error = 0;
    if ( (n = connect(sockfd, saptr, salen)) < 0)
        if (errno != EINPROGRESS)
            return(-1);

    /* Do whatever we want while the connect is taking place. */

    if (n == 0)
        goto done;	/* connect completed immediately */

    FD_ZERO(&rset);
    FD_SET(sockfd, &rset);
    wset = rset;
    tval.tv_sec = nsec;
    tval.tv_usec = 0;

    if ( select(sockfd+1, &rset, &wset, NULL,
                nsec ? &tval : NULL) <= 0) {
        //close(sockfd);		/* timeout */
        errno = ETIMEDOUT;
        return(-1);
    }

    if (FD_ISSET(sockfd, &rset) || FD_ISSET(sockfd, &wset)) {
        len = sizeof(error);
        if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &error, &len) < 0)
            return(-1);			/* Solaris pending error */
    } else
        ROS_INFO("select error: sockfd not set\n");
done:
    TCPFcntl(sockfd, F_SETFL, flags);	/* restore file status flags */

    if (error) {
        //close(sockfd);		/* just in case */
        errno = error;
        return(-1);
    }
    return(0);
}

int TCPSocketAPI::tcpAccept()
{
    struct sockaddr_in cli_addr;
    socklen_t clilen;
    bzero(&cli_addr, sizeof(cli_addr));
    clilen = sizeof(cli_addr);
    if(m_clientSocket > 0) return 1 ;
    m_clientSocket = accept(m_sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if (m_clientSocket < 0) {
        ROS_INFO("couldn't accept errno %d m_sockfd %d\n",errno,m_sockfd);
        m_clientSocket = 0 ;
        return -1;
    }
    ROS_INFO("accept ok\n");
    return 2;

}

void TCPSocketAPI::closeSocket()
{
    if(m_sockfd) {
        close(m_sockfd);
        m_sockfd = 0 ;
    }

}
////end read




}//end namespace
