using Sockets

udpsock = UDPSocket()

bind(udpsock, ip"192.168.178.25", 5555)
while true
  println(String(copy(recv(udpsock))))
end

