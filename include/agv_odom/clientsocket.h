/*
 * clientsocket.h
 *
 *  Created on: Jul 29, 2016
 *      Author: paul
 */

#ifndef INCLUDE_CLIENTSOCKET_H_
#define INCLUDE_CLIENTSOCKET_H_
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/exception/all.hpp>

//typedef boost::error_info<struct tag_err_no, int> err_no;
using namespace boost::asio::ip;
/*typedef enum SocketState
{
    unoccupied,
    connecting,
    connected,
    connect_fail,
    receive_fail,
    send_fail
}SOCSTATE;*/

void wait(int milliseconds);

class SocketException
{
public:
  enum SocketState
  {
      unoccupied,
      connecting,
      connected,
      connect_fail,
      receive_fail,
      receiving,
      send_fail,
      sending
  };
  SocketException();
  ~SocketException();
  void stateSet(SocketState &soc_state);
  SocketState getState() const {return state_;}
private:
  void checkState();
protected:
  SocketState state_;
  boost::mutex state_mutex_;
private:
  boost::shared_ptr<boost::thread> state_check_;                              //怎么添加状态监控机制？
};

class ClientSocket:public SocketException
{
    public:
      ClientSocket(boost::asio::io_service &io_service, tcp::endpoint &endpoint, const unsigned int recv_buffer_size);
      ~ClientSocket()
      {
        std::cout<<"~ClientSocket()" << std::endl;
        client_socket_.shutdown(client_socket_.shutdown_both);
      }
    public:
  //    void socketRead(char  recv_buffer[], unsigned int &recv_bytes);
      void socketRead();
      void socketWrite(const char* write_buffer, const unsigned int &size);

    private:
      bool reset();
      tcp::socket client_socket_;              //Client socket.
      tcp::endpoint endpoint_;
      boost::shared_ptr<boost::thread> read_thrd_ptr;
      unsigned int recv_buffer_size_;

    public:
      //shared data.
      static char recv_buffer_[1700];                                       // The quick cache.
      static unsigned int recv_bytes_;                                      //
};

#endif /* INCLUDE_CLIENTSOCKET_H_ */
