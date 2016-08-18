/*
 * clientsocket.cpp
 *
 *  Created on: Jul 29, 2016
 *      Author: paul
 */
#include <iostream>

#include <boost/bind.hpp>
//#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/chrono.hpp>

#include "agv_odom/clientsocket.h"
#include "agv_odom/data_definition.h"
extern boost::shared_mutex mutex;                                                      //A global mutex used in this project.
//extern boost::mutex read_mutex;
/******************************************/
/*                               SocketException                      */
/*****************************************/
SocketException::SocketException():state_(unoccupied)
{
  std::cout<<"SocketException()" << std::endl;
  state_check_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&SocketException::checkState, this)));
}
SocketException::~SocketException()
{
  std::cout<<"~SocketException()" << std::endl;
}

void SocketException::checkState()
{
  while(true)
  {
  //  wait(10);                   //10ms per check.
    state_mutex_.lock();
    switch (state_) {
      case unoccupied:
//        std::cout << "Socket is UNOCCUPIED." << std::endl;
        break;
      case connecting:
//        std::cout << "Socket is CONNECTING..." << std::endl;
        break;
      case connected:
//        std::cout << "Socket is CONNECTED." << std::endl;
        break;
      case connect_fail:
        std::cout << "Socket is CONNECT FAIL." << std::endl;
        break;
      case receive_fail:
        std::cout << "Socket is READ FAIL." << std::endl;
        break;
      case receiving:
//        std::cout << "Socket is RECEIVING." << std::endl;
        break;
      case send_fail:
        std::cout << "Socket is SEND FAIL." << std::endl;
        break;
      case sending:
//        std::cout << "Socket is SENDING." << std::endl;
        break;
      default:
        break;
    }
    state_mutex_.unlock();
  }
}
/******************************************/
/*                               ClientSocket                            */
/*****************************************/

char ClientSocket::recv_buffer_[1700] = {0};
unsigned int ClientSocket::recv_bytes_ = 0;
ClientSocket::ClientSocket(boost::asio::io_service &io_service, tcp::endpoint &endpoint, const unsigned int recv_buffer_size):
client_socket_(io_service),endpoint_(endpoint), recv_buffer_size_(recv_buffer_size)
{
  std::cout<<"ClientSocket()" << std::endl;
        //==============================
        //  Socket option for the receive buffer size of a socket.
        //==============================
/*        boost::asio::socket_base::receive_buffer_size option(8192);
          client_socket_.set_option(option);*/
   //==============================
   //                   同步
   //==============================
        boost::system::error_code error;
        client_socket_.open(boost::asio::ip::tcp::v4());

//      boost::asio::socket_base::non_blocking_io command(true);
//      boost::asio::ip::tcp::socket::non_blocking_io command(false);
//      client_socket_.io_control( command);
        state_mutex_.lock();
        state_ = connecting;
        state_mutex_.unlock();

        if(client_socket_.connect(endpoint_,  error))                              //connect to server.[Blocking until connection is successful or an error occurs.]
        {
          try                                                                                          //try until time out.
          {
             std::cout << "connect fail.Try connect..."<< std::endl;
             client_socket_.connect(endpoint_);                                 //try to connect once again.
          }
          catch(boost::exception &exception)                                      //throw the error information.
          {
            throw boost::system::system_error(error);
            client_socket_.shutdown(client_socket_.shutdown_both);
            std::cout << "client shutdown" << std::endl;
          }
        }
        else
        {
          //==============================
          //  Getting the current option value:
          //==============================
//          boost::asio::socket_base::receive_buffer_size option;
//          client_socket_.get_option(option);
//          std::cout << option.value() << std::endl;
            std::cout << "Client socket connect to server successfully." << std::endl;

            state_mutex_.lock();
            state_ = connected;
            state_mutex_.unlock();

            read_thrd_ptr = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ClientSocket::socketRead, this )));                  //read thread.
 //         client_socket_.keep_alive;          //怎么使用？
        }
}

//void ClientSocket::socketRead(char  recv_buffer[], unsigned int &recv_bytes)
void ClientSocket::socketRead()
{
      boost::system::error_code read_error;
      char buffer[recv_buffer_size_];
//      boost::this_thread::interruption_point();
      while(true)
      {
          memset((char*)buffer, 0, recv_buffer_size_);                                                           //Initializing the temporary buffer.
          boost::unique_lock<boost::shared_mutex> lock(mutex);                                    //A thread modifying the share buffer needs write access and thus requires an exclusive lock.
     //     read_mutex.lock();
          recv_bytes_  = client_socket_.read_some(boost::asio::buffer((char *)buffer, recv_buffer_size_), read_error);   //   * This function is used to read data from the stream socket. The function
                                                                                                                                                                                      //   * call will BLOCK until one or more bytes of data has been read successfully,
                                                                                                                                                                                      //   * or until an error occurs.
          if(!read_error)                                                                                                            // if socket read successfully, then we can get data safely.
          {
            memset((char*)recv_buffer_, 0, recv_buffer_size_);                                            //再每次接收之前需要清空吗？
            std::memcpy((char*) recv_buffer_, (char*)buffer, recv_bytes_);                           //memory copy.
//            std::cout <<recv_bytes_ << std::endl;
            lock.unlock();                                                                                                        // should manually release the lock, let another threads can get the mutex.
//              read_mutex.unlock();
            state_mutex_.lock();
            state_ = receiving;
            state_mutex_.unlock();
          }
          else
          {
            throw boost::system::system_error(read_error);                                                   //throw the error information.
            try                                                                                                                         //try until time out.
            {
              std::cout << "socket try connecting..."<< std::endl;
              client_socket_.connect(endpoint_);                                     //try to connect once again.
              state_mutex_.lock();
              state_ = connecting;
              state_mutex_.unlock();
            }
            catch(boost::exception &exception)                                           //throw the error information.
            {
              state_mutex_.lock();
              state_ = connect_fail;
              state_mutex_.unlock();
//                std::cerr << *get_error_info<err_no>(exception) << std::endl;
              throw boost::system::system_error(read_error);
              client_socket_.shutdown(client_socket_.shutdown_both);
            }
            }
          wait(20);       //20ms->50HZ
      }
//    boost::asio::io_service io_service;
//    boost::asio::deadline_timer timer(io_service, boost::posix_time::milliseconds(100));        //10HZ
//    timer.wait();
}

void ClientSocket::socketWrite(const char* write_buffer, const unsigned int &size)
{
      boost::system::error_code write_error;
      client_socket_.write_some(boost::asio::buffer((char*)write_buffer, size), write_error);
      if(write_error)                                                                                //write error.
      {
        state_mutex_.lock();
        state_ = send_fail;
        state_mutex_.unlock();
        std::cout << "Socket write fail." << std::endl;
        throw boost::system::system_error(write_error);
        client_socket_.shutdown(client_socket_.shutdown_send);        //shutdown the pipe of sending message.
        if(reset())
          client_socket_.write_some(boost::asio::buffer((char*)write_buffer, size), write_error);   //
      }

      state_mutex_.lock();
      state_ = sending;
      state_mutex_.unlock();

  //    return true;              //actually, we can't know whether the message is send successfully.
}

bool ClientSocket::reset()
{
      boost::system::error_code error;
      if(client_socket_.is_open())                               //if socket is alive still, we will close it.
        client_socket_.cancel();

      if(!client_socket_.connect(endpoint_,  error))     //connect to sever.
        return true;
      else
      {
          std::cout << "Socket reset fail." << std::endl;
          throw boost::system::system_error(error);
      }
      return false;
}

// sleep function
void wait(int milliseconds)
{
 //   boost::this_thread::sleep_for(boost::chrono::seconds(seconds));
    boost::this_thread::sleep_for(boost::chrono::milliseconds(milliseconds));
}
