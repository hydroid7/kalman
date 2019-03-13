using Sockets, JSON

"""
    @loop expression

Repeats the `expression` forever
"""
macro loop(ex)
    quote
        while true
            $(esc(ex))
        end
    end
end

struct Accelerometer
    x
    y
    z
end

struct Gyroskope
    x
    y
    z
end

struct Magnetic
    x
    y
    z
end

calculatePosition(acceleration, time, vPrev) = vPrev + 1//2 * acceleration * time^2

function takeInput!(to::Channel)
    @async begin
        udpsock = UDPSocket()
        bind(udpsock,ip"192.168.178.25", 5555)
        @loop begin
            put!(to, String(recv(udpsock)))
        end
    end
end

function parseInput!(from::Channel, to::Channel)
    @async @loop begin
        data = take!(from)
        data = "(" * data
        data = replace(data, ", 3, " => ", Accelerometer(")
        data = replace(data, ", 4, " => "), Gyroskope(")
        data = replace(data, ", 5," => "), Magnetic(")
        data = data * "))"
        # (Timestamp(7691.43761), Accelerometer(-0.067, -0.062, 9.73), Gyroskope(-0.0, 0.001, 0.001), Magnetic(43.9, -12.3, -117.0))
        row = eval(Meta.parse(data))
        put!(to, row)
    end
end

function sendToRenderer!(from::Channel, bindPort::Int64)
    @async begin
        outputSocket = UDPSocket()
        @loop begin
            send(outputSocket, ip"127.0.0.1", bindPort, json(take!(from)))
        end
    end
end

function calculatingChannel(from::Channel, to::Channel)
    @async begin
        local prevLine = fetch(from)
        @loop begin
            task = take!(from)

            # TODO Calculate Kalman

            prevLine = task
            put!(to, out)
        end
    end
end

inputChanel = Channel(16) # Receives the input
parsedChannel = Channel(16) # Receives parsed data
outputChannel = Channel(16) # It's content is pushed to the rust server.
calculatingChannel = Channel(16)

takeInput!(inputChanel)
parseInput!(inputChanel, calculatingChannel)
calculateInput!(calculatingChannel, outputChannel)
sendToRenderer!(outputChannel, 2000)

while running
    sleep(10)
end
