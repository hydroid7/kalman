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

struct Vector3
    x::Number
    y::Number
    z::Number
end
Vector3(dict::Dict) = Vector3(dict["x"], dict["y"], dict["z"])

struct GPS
    lat
    lon
end

function getOrDefault(array, default)
    if length(array) == 1
        array[1]
    else
        default
    end
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
    prev = (timestamp = 0.0, accelerometer = Vector3(0, 0, 0), gyro = Vector3(0, 0, 0), magneto = Vector3(0, 0, 0))
    @async @loop begin
        data = take!(from)
        # Generate Tuple from incoming data
        data = "[" * data[1:end-2] * "]"
        data = JSON.parse(data)
        timestamp = data[1]
        data = data[2:end]
        # Accelerometer
        accelerometer = filter(x -> x["sensor_id"] == "accelerometer", data)
        @show accelerometer
        if length(accelerometer) == 1
            accelerometer = Vector3(accelerometer[1])
        else
            accelerometer = missing
        end

        # Gyroskope
        gyro = filter(x -> x["sensor_id"] == "gyroskope", data)
        if length(gyro) == 1
            gyro = Vector3(gyro[1])
        else
            gyro = missing
        end

        # Magnetometer
        magneto = filter(x -> x["sensor_id"] == "magnetometer", data)
        if length(magneto) == 1
            magneto = Vector3(magneto[1])
        else
            magneto = missing
        end

        # The result is a named tuple with all values
        result = (timestamp = timestamp, accelerometer = accelerometer, gyro = gyro, magneto = magneto)
        prev = result
        @show result
        put!(to, result)
    end
end

function calculateInput!(from::Channel, to::Channel)
    @async @loop begin
        task = take!(from)
        # TODO Calculate Kalman
        out = Dict(
            "timestamp" => task[1],
            "position" => Dict("x" => 1, "y" => 2, "z" => 3)
        )
        prevLine = task
        put!(to, out)
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

inputChanel = Channel(16) # Receives the input
parsedChannel = Channel(16) # Receives parsed data
outputChannel = Channel(16) # It's content is pushed to the rust server.
calculatingChannel = Channel(16)

takeInput!(inputChanel)
parseInput!(inputChanel, calculatingChannel)
# calculateInput!(calculatingChannel, outputChannel)
# sendToRenderer!(outputChannel, 2000)

@loop sleep(10)
